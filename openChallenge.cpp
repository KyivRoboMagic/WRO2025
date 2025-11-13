#include <iostream>
#include <pigpio.h>
#include <opencv2/opencv.hpp>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <thread>
#include <atomic>

#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <mutex>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#pragma GCC optimize("Ofast")
//#pragma GCC target("avx,avx2,tune=native")

#define SERVO_PWM 12
#define MOTOR_PWM 13
#define MOTOR_DIR 17

#define BUTTON_PIN 24

using namespace sl;

namespace LidarStore {
    static sl_lidar_response_measurement_node_hq_t bufA[8192];
    static sl_lidar_response_measurement_node_hq_t bufB[8192];
    static std::atomic<size_t> countA{0}, countB{0};
    static std::atomic<int> active_idx{0}; // 0 = A active, 1 = B active
    static std::atomic<uint64_t> scan_idx{0};

    inline const sl_lidar_response_measurement_node_hq_t* get_latest(size_t& out_count, uint64_t& id) {
	id = scan_idx.load(std::memory_order_acquire);

        int idx = active_idx.load(std::memory_order_acquire);
        out_count = (idx==0) ? countA.load(std::memory_order_relaxed)
                             : countB.load(std::memory_order_relaxed);
        return (idx==0) ? bufA : bufB;
    }
}

namespace CameraStore {
    static cv::Mat bufA(480, 640, CV_8UC3);
    static cv::Mat bufB(480, 640, CV_8UC3);
    static std::atomic<int> active_idx{0}; // 0 = A active, 1 = B active
    static std::atomic<uint64_t> frame_idx{0};

    inline cv::Mat& get_inactive_buffer() {
        int idx = active_idx.load(std::memory_order_relaxed);
        return (idx == 0) ? bufB : bufA;
    }

    inline void publish() {
        int idx = active_idx.load(std::memory_order_relaxed);
        active_idx.store(1 - idx, std::memory_order_release);
        frame_idx.fetch_add(1, std::memory_order_relaxed);
    }

    inline const cv::Mat* get_latest(uint64_t& id) {
        int idx = active_idx.load(std::memory_order_acquire);
        id = frame_idx.load(std::memory_order_relaxed);
        return (idx == 0) ? &bufA : &bufB;
    }
}

static const auto T0 = std::chrono::steady_clock::now();
static inline uint64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - T0).count();
}

std::atomic<bool> running{true};
void onSigint(int) { running = false; }

size_t nodes_count = 0;
uint64_t scan_id = 0, scan_id_old = 0;
uint64_t frame_id = 0, frame_id_old = 0;

bool direction = true; // true - clockwise

sl_lidar_response_measurement_node_hq_t nodes[8192];

cv::Mat m_map(620, 620, CV_8UC1);
uchar* map = m_map.data;

cv::Mat frame(240, 640, CV_8UC3);
cv::Mat image(240, 640, CV_8UC3);
cv::Mat hsv(240, 640, CV_8UC3);
cv::Mat mask(240, 640, CV_8UC1);
uchar* mask_p = mask.data;

cv::Mat m_draw(620, 620, CV_8UC1);
uchar* draw = m_draw.data;

int missMatchCounter = 0, ChangeZoneThreshould = 3, zone = 0;
bool zoneIsCorner = 0;

bool endgame = false;
double endTimer;

std::vector<std::array<int, 3> > scanXY;
std::vector<std::array<float, 3> > scanMA;

std::vector<cv::Vec4i> lines_raw;
std::vector<std::pair<float, cv::Vec4i> > lines;

int buttonSum = 0;

bool is_button_down() {
    if (gpioRead(BUTTON_PIN) == 0) {
        return true;
    } else {
        return false;
    }
}

void fill(uchar* ptr, int val, int x, int y, int x0, int y0, int w, int h) {
    if (ptr[x + w * y] == val) return;
    ptr[x + w * y] = val;
    if (x0 < x) fill(ptr, val, x-1, y, x0, y0, w, h);
    if (y0 < y) fill(ptr, val, x, y-1, x0, y0, w, h);
    if (x < x0 + w - 1) fill(ptr, val, x+1, y, x0, y0, w, h);
    if (y < y0 + h - 1) fill(ptr, val, x, y+1, x0, y0, w, h);
    return;
}

void setup_GPIO() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio setup failed!\n";
        exit(1);
    }
    gpioSetMode(SERVO_PWM, PI_OUTPUT);
    gpioSetMode(MOTOR_PWM, PI_OUTPUT);
    gpioSetMode(MOTOR_DIR, PI_OUTPUT);
    
    gpioSetMode(BUTTON_PIN, PI_INPUT);
    gpioSetPullUpDown(BUTTON_PIN, PI_PUD_UP);
}

void servo(float angle) {
    int pulseWidth = (angle * 2000) + 500;
    gpioServo(SERVO_PWM, pulseWidth);
}

void motor(float speed) {
    speed *= 255;

    if (speed > 0) {
        gpioWrite(MOTOR_DIR, PI_HIGH);
    } else if (speed < 0) {
        speed = -speed;
        gpioWrite(MOTOR_DIR, PI_LOW);
    }

    if (speed > 255) speed = 255;

    gpioPWM(MOTOR_PWM, speed);
}

void printDeviceInfo(ILidarDriver* drv){
    sl_lidar_response_device_info_t info;
    if (SL_IS_OK(drv->getDeviceInfo(info))) {
        std::cerr << "Model " << (int)info.model
                  << " | FW " << ((info.firmware_version>>8)&0xFF) << "."
                  << (info.firmware_version & 0xFF)
                  << " | HW " << (int)info.hardware_version
                  << " | SN ";
        for (int i=0;i<16;++i) std::cerr << std::hex << (int)info.serialnum[i];
        std::cerr << std::dec << "\n";
    } else {
        std::cerr << "getDeviceInfo failed\n";
    }
}

bool checkHealth(ILidarDriver* drv){
    sl_lidar_response_device_health_t h;
    sl_result r = drv->getHealth(h);
    if (!SL_IS_OK(r)) {
        std::cerr << "getHealth failed, code=0x" << std::hex << r << std::dec << "\n";
        return false;
    }
    std::cerr << "Health status: " << (int)h.status << "\n";
    if (h.status == SL_LIDAR_STATUS_ERROR) {
        std::cerr << "Internal error reported. Power-cycle or reset.\n";
        return false;
    }
    return true;
}

void scanThread(ILidarDriver* drv){
    sl_lidar_response_measurement_node_hq_t local[8192];

    while (running.load(std::memory_order_relaxed)) {
        size_t count = sizeof(local)/sizeof(local[0]);
        sl_result res = drv->grabScanDataHq(local, count);
        if (!SL_IS_OK(res)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        drv->ascendScanData(local, count);

	LidarStore::scan_idx.fetch_add(1, std::memory_order_relaxed);

        int cur = LidarStore::active_idx.load(std::memory_order_relaxed);
        if (cur == 0) {
            std::memcpy(LidarStore::bufB, local, count * sizeof(local[0]));
            LidarStore::countB.store(count, std::memory_order_relaxed);
            LidarStore::active_idx.store(1, std::memory_order_release);
        } else {
            std::memcpy(LidarStore::bufA, local, count * sizeof(local[0]));
            LidarStore::countA.store(count, std::memory_order_relaxed);
            LidarStore::active_idx.store(0, std::memory_order_release);
        }
    }
}

void cameraThread(cv::VideoCapture* cap) {
    while (running.load(std::memory_order_relaxed)) {
        cv::Mat& dst = CameraStore::get_inactive_buffer();
        if (cap->read(dst)) {
            CameraStore::publish();
        }
    }
}

double angle_between_lines(const cv::Vec4i& l1, const cv::Vec4i& l2) {
    cv::Point2d d1(l1[2] - l1[0], l1[3] - l1[1]);
    cv::Point2d d2(l2[2] - l2[0], l2[3] - l2[1]);

    double dot = d1.x * d2.x + d1.y * d2.y;
    double norm1 = std::hypot(d1.x, d1.y);
    double norm2 = std::hypot(d2.x, d2.y);

    if (norm1 == 0 || norm2 == 0) return 0.0;

    double cosTheta = dot / (norm1 * norm2);
    cosTheta = std::max(-1.0, std::min(1.0, cosTheta));

    double angle = std::acos(cosTheta) * 180.0 / CV_PI;

    return angle;
}

int center_distance(const cv::Vec4i& l1, const cv::Vec4i& l2) {
    cv::Point2d c1((l1[0] + l1[2]) / 2, (l1[1] + l1[3]) / 2);
    cv::Point2d c2((l2[0] + l2[2]) / 2, (l2[1] + l2[3]) / 2);
    return sqrt((c1.x - c2.x) * (c1.x - c2.x) + (c1.y - c2.y) * (c1.y - c2.y));
}

void process_hsv(uchar*& hsv_p, uchar*& red_p) {
    int hue, sat, val;
    for (int p = 0; p < 240 * 640; p++) {
        red_p[p] = 0;

        hue = hsv_p[p * 3    ];
        sat = hsv_p[p * 3 + 1];
        val = hsv_p[p * 3 + 2];

        if (val < 60) {
            red_p[p] = 255;
        }
    }
}

double pointLineDistance(const cv::Point2d& p, const cv::Vec4i& line) {
    double x1 = line[0], y1 = line[1];
    double x2 = line[2], y2 = line[3];

    double num = std::abs((x2 - x1)*(y1 - p.y) - (x1 - p.x)*(y2 - y1));
    double den = std::hypot(x2 - x1, y2 - y1);
    return den < 1e-9 ? 0.0 : num / den;
}

cv::Point2d middlePoint(const cv::Vec4i& line) {
    return cv::Point2d((line[2] + line[0]) >> 1, (line[3] + line[1]) >> 1);
}

int pointDistance(cv::Point2d a, cv::Point2d b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

inline cv::Point2d lineIntersection(const cv::Vec4i& a, const cv::Vec4i& b) {
    cv::Point2d p1(a[0], a[1]), p2(a[2], a[3]);
    cv::Point2d q1(b[0], b[1]), q2(b[2], b[3]);

    double A1 = p2.y - p1.y;
    double B1 = p1.x - p2.x;
    double C1 = A1 * p1.x + B1 * p1.y;

    double A2 = q2.y - q1.y;
    double B2 = q1.x - q2.x;
    double C2 = A2 * q1.x + B2 * q1.y;

    double det = A1 * B2 - A2 * B1;

    // since lines always intersect, det ` 0
    return cv::Point2d(
        (B2 * C1 - B1 * C2) / det,
        (A1 * C2 - A2 * C1) / det
    );
}

void process_raw_frame(uchar* raw_frame_p, uchar*& frame_p) {
    int p1 = 0, p2 = 640 * 240 * 3;
    for (int y = 240; y < 480; ++y) {
	for (int x = 0; x < 640; ++x) {
	    frame_p[p1] = raw_frame_p[p2];
	    frame_p[p1 + 1] = raw_frame_p[p2 + 1];
	    frame_p[p1 + 2] = raw_frame_p[p2 + 2];

	    p1 += 3;
	    p2 += 3;
	}
    }
}

void grayWorldWB(const cv::Mat& bgr, cv::Mat& bgrOut) {
    CV_Assert(bgr.type() == CV_8UC3);
    cv::Scalar m = cv::mean(bgr); // B,G,R
    double gray = (m[0] + m[1] + m[2]) / 3.0;
    cv::Mat gain = (cv::Mat_<double>(1,3) << gray/(m[0]+1e-6),
                                             gray/(m[1]+1e-6),
                                             gray/(m[2]+1e-6));
    bgr.convertTo(bgrOut, CV_32F);
    cv::multiply(bgrOut, gain, bgrOut);
    cv::Mat tmp;
    cv::normalize(bgrOut, tmp, 0, 255, cv::NORM_MINMAX);
    tmp.convertTo(bgrOut, CV_8U);
}

long long RayToLineIntersectLength(cv::Vec4i a, const cv::Point2d b, double angle) {
    a[0] -= b.x;
    a[1] -= b.y;
    a[2] -= b.x;
    a[3] -= b.y;

    double m1 = std::hypotf(float(a[0]), float(a[1]));
    double m2 = std::hypotf(float(a[2]), float(a[3]));

    double a1 = std::atan2(float(a[1]), float(a[0])) - angle;
    double a2 = std::atan2(float(a[3]), float(a[2])) - angle;

    a[0] = m1 * std::cos(a1);
    a[1] = m1 * std::sin(a1);
    a[2] = m2 * std::cos(a2);
    a[3] = m2 * std::sin(a2);


    cv::Point2d p1(a[0], a[1]), p2(a[2], a[3]);

    double A1 = p2.y - p1.y;
    double B1 = p1.x - p2.x;
    double C1 = A1 * p1.x + B1 * p1.y;

    if (B1 == 0) return -1; // det = -B1

    long long y = C1 / B1;

    if (y > 0) return -1;
    return -y;
}

void nextCorner(int& x, int &y, int idx) {
    x = 0;
    y = 0;
    for (int j = 0; j < lines.size(); j++) if (j != idx) {
	double angle = angle_between_lines(lines[idx].second, lines[j].second);
	if (angle > 90.0) angle = 180.0 - angle;
	if (angle > 80) {
	    cv::Point intersect = lineIntersection(lines[idx].second, lines[j].second);
	    if (intersect.x > x) {
		x = intersect.x;
		y = intersect.y;
	    }
	}
    }
    return;
}

void findDirection() {
    while (running.load(std::memory_order_relaxed)) {
	const auto* nodes = LidarStore::get_latest(nodes_count, scan_id);
	if (scan_id > 2) {
	    std::vector <long long> sum(361, 0);
	    for (int i = 0; i < nodes_count; i++) if (nodes[i].dist_mm_q2 != 0) {
		float angle = nodes[i].angle_z_q14 * 90.f / 16384.f * M_PI / 180;
		float dist = nodes[i].dist_mm_q2 / 40.f;
		int idx = angle * 180 / M_PI;
		sum[idx+1] += dist;
	    }
	    for (int i = 1; i <= 360; i++) sum[i] += sum[i-1];

	    long long sumAt45 = sum[45+15] - sum[45-16];
	    long long sumAt135 = sum[135+15] - sum[135-16];
	    long long sumAt225 = sum[225+15] - sum[225-16];
	    long long sumAt315 = sum[315+15] - sum[315-16];
	    if (sumAt135 + sumAt315 <= sumAt45 + sumAt225) {
		std::cout << "CLOCKWISE\n";
		direction = true;
	    } else {
		std::cout << "COUNTERCLOCKWISE\n";
		direction = false;
	    }
	    break;
	}
    }
}

void mergeAllLines() {
    for (auto it1 = lines.begin(); it1 != lines.end(); ++it1) {
	auto it2 = std::next(it1);
	while (it2 != lines.end()) {
	    double angle = angle_between_lines(it1->second, it2->second);
	    if (angle > 90.0) angle = 180.0 - angle;
	    if (angle < 30.0 && pointLineDistance(cv::Point((it2->second[2] + it2->second[0])/2, (it2->second[3] + it2->second[1]) / 2), it1->second) < 10) {
		cv::Point points[4];
		points[0] = {it1->second[0], it1->second[1]};
		points[1] = {it1->second[2], it1->second[3]};
		points[2] = {it2->second[0], it2->second[1]};
		points[3] = {it2->second[2], it2->second[3]};
		int best_length = it1->first;
		int best_x = 0, best_y = 1;
		for (int i = 0; i < 3; i++) {
		    for (int j = i + 1; j < 4; j++) {
			int current_l = std::hypot(points[i].x - points[j].x, points[i].y - points[j].y);
			if (current_l > best_length) best_x = i, best_y = j, best_length = current_l;
		    }
		}
		it1->second[0] = points[best_x].x;
		it1->second[1] = points[best_x].y;
		it1->second[2] = points[best_y].x;
		it1->second[3] = points[best_y].y;

		it2 = lines.erase(it2);
	    } else {
		++it2;
	    }
	}
	it1->first = std::hypot((it1->second[2] - it1->second[0]), (it1->second[3] - it1->second[1]));
    }
}
int main() {
    const char* port = "/dev/ttyUSB0";
    int baud = 1000000;

    std::signal(SIGINT, onSigint);

    setup_GPIO();

    motor(0);

    const std::string pipeline =
        "libcamerasrc ! "
        "video/x-raw,width=640,height=480,framerate=100/1 ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "queue max-size-buffers=1 leaky=downstream ! "
        "appsink drop=true max-buffers=1 sync=false";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) { std::cerr << "camera open failed\n"; return 2; }

    ILidarDriver* drv = *createLidarDriver();
    if (!drv) { std::cerr << "createLidarDriver failed\n"; return 3; }

    IChannel* ch = *createSerialPortChannel(port, baud);
    if (!ch) {
	std::cerr << "createSerialPortChannel failed for " << port << "\n";
	delete drv; return 4;
    }

    if (SL_IS_FAIL(drv->connect(ch))) {
	std::cerr << "connect failed: " << port << " @ " << baud << "\n";
	delete ch; delete drv; return 5;
    }

    printDeviceInfo(drv);
    if (!checkHealth(drv)) { drv->disconnect(); delete ch; delete drv; return 6; }

    drv->setMotorSpeed();

    if (SL_IS_FAIL(drv->startScan(false, true))) {
	std::cerr << "startScan failed\n"; drv->setMotorSpeed(0);
	drv->disconnect(); delete ch; delete drv; return 7;
    }

    buttonSum = 0;
    while (buttonSum < 50) {
	if (is_button_down()) buttonSum++;
	else buttonSum--;
	if (buttonSum < 0) buttonSum = 0;
    }
    buttonSum = 0;

    std::thread lidar_thread(scanThread, drv);
    std::thread camera_thread(cameraThread, &cap);
    
    servo(0.48);

    findDirection();
    
    double loop_t0 = now_ns();
    
    //*
    for (double i = 0; i <= 0.6; i += 0.05) {
	double at0 = now_ns();
	while ((now_ns() - at0) * 1e-6 < 500.0 / 12) {}
	motor(i);
    }//*/
    
    while (running.load(std::memory_order_relaxed)) {
	const cv::Mat* raw_frame = CameraStore::get_latest(frame_id);
	const auto* nodes = LidarStore::get_latest(nodes_count, scan_id);

	if (scan_id != scan_id_old) {
	    scanXY.clear();
	    scanMA.clear();
	    lines_raw.clear();
	    lines.clear();
	    for (int i = 0; i < 620 * 620; i++) map[i] = 0;
	    
	    for (int i = 0; i < nodes_count; i++) if (nodes[i].dist_mm_q2 != 0) {
		float angle = nodes[i].angle_z_q14 * 90.f / 16384.f * M_PI / 180;
		float dist = nodes[i].dist_mm_q2 / 40.f;

		scanMA.push_back({dist, angle, float(nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT)});

		int x = std::cos(angle) * dist;
		int y = std::sin(angle) * dist;
		x = std::min(std::max(x+310, 0), 619);
		y = std::min(std::max(y+310, 0), 619); // update!

		scanXY.push_back({x, y, nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT});

		map[y*620+x]=255;
		if (x != 619) map[y*620+x+1]=255;
		if (x != 0) map[y*620+x-1]=255;
		if (y != 619) map[y*620+x+620]=255;
		if (y != 0) map[y*620+x-620]=255;
	    }

	    HoughLinesP(m_map, lines_raw, 1.0, (float)CV_PI/180.0f, 10, 10.0, 20.0);

	    for (cv::Vec4i line : lines_raw) {
		lines.push_back({
		    std::hypot((line[2] - line[0]), (line[3] - line[1])),
		    line
		});
	    }

	    mergeAllLines();

	    std::sort(lines.begin(), lines.end(), [](const auto &a, const auto &b) { return a.first > b.first; }); // reversed

	    int idx = -1; long long len = 2e18;
	    for (int i = 0; i < lines.size(); i++) {
		long long len_current = RayToLineIntersectLength(lines[i].second, cv::Point2d(310, 310), 0.0);

		if (len_current == -1) continue;
		if (len > abs(len_current)) {
		    idx = i;
		    len = abs(len_current);
		}
	    }
	    
	    int x, y;
	    nextCorner(x, y, idx);
	    
	    double dist = std::hypot(x - 310, y - 310);

	    bool detectedZone = (dist < 120);

	    if (zoneIsCorner == detectedZone) {
		missMatchCounter = 0;
	    } else {
		missMatchCounter++;
		if (missMatchCounter >= ChangeZoneThreshould) {
		    zoneIsCorner = detectedZone;
		    zone++;
		    missMatchCounter = 0;
		    std::cout << "Now in zone " << zone << "\n";
		}
	    }
	}

	if (frame_id != frame_id_old) {
	    process_raw_frame(raw_frame->data, frame.data);
	    grayWorldWB(frame, image);
	    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
	    process_hsv(hsv.data, mask.data);
	    
	    int l = 0, r = 0, m_th = 120;
	    for (int i = 0; i < 240; i++) {
		for (int j = 0; j < 640; j++) {
		    if (mask_p[640 * i + j] == 255) {
			if (j < 320 - m_th) l++;
			else if (320 + m_th < j) r++;
		    }
		}
	    }
	    int err = 0, errold = 0;
	    double dir;
	    if (direction) {
		err = l-20000;
	    } else {
		err = 20000-r;
	    }
	    double kp1 = 0.00005;
	    dir = 0.48 + err * kp1;
	    dir = std::min(std::max(dir, 0.42), 0.54);
	    //std::cout << l << " " << r << " " << dir << " " << (ct1 - ct0) * 1e-6 << "\n";
	    servo(dir);
	    errold = err;
	}

	scan_id_old = scan_id;
	frame_id_old = frame_id;

	if (zone >= 23 && !endgame) {
	    endgame = true;
	    endTimer = now_ns();
	}
	if (endgame && now_ns() - endTimer >= 2000e6) running = false;

	if (now_ns() - loop_t0 >= 180e9) running = false;
	
	if (is_button_down()) buttonSum++;
	else buttonSum--;
	if (buttonSum < 0) buttonSum = 0;
	if (buttonSum > 100) running = false;
    }
    motor(0);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    cv::Point center_point(310, 310);
    cv::circle(m_map, center_point, 5, 200, 2);
    cv::imwrite("lidar.png", m_map);
    cv::imwrite("frame.png", frame);
    cv::imwrite("image.png", image);
    cv::imwrite("mask.png", mask);


    running = false;
    drv->stop();
    if (lidar_thread.joinable()) lidar_thread.join();
    if (camera_thread.joinable()) camera_thread.join();
    drv->setMotorSpeed(0);
    drv->disconnect();
    delete ch;
    delete drv;

    return 0;
}
