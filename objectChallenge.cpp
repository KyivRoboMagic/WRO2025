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


#pragma pack(push,1)
struct ImuHeader { uint8_t type, ver; uint16_t rsv; uint64_t t_ns; uint32_t seq; };
struct ImuPacket {
    ImuHeader h;
    float ax, ay, az;                  // linear accel (body)
    float qi, qj, qk, qr;              // quaternion (x,y,z,w)
    float gz;                          // gyro Z (rad/s) in body frame
};
#pragma pack(pop)

struct IMUSample {
    uint64_t t_ns;
    uint32_t seq;
    float ax, ay, az;
    float qi, qj, qk, qr;
    float gz; // rad/s
};

class IMUBuffer {
public:
    void reserve(size_t n) { std::lock_guard<std::mutex> lk(m_); buf_.reserve(n); }
    void push(const IMUSample& s) {
        std::lock_guard<std::mutex> lk(m_);
        buf_.emplace_back(s);
    }
    std::vector<IMUSample> take_and_clear() {
        std::lock_guard<std::mutex> lk(m_);
        std::vector<IMUSample> out;
        out.swap(buf_);                  // O(1) move, clears internal buffer
        if (buf_.capacity() < reserve_)  // keep a baseline reserve to avoid churn
            buf_.reserve(reserve_);
        return out;
    }
private:
    std::mutex m_;
    std::vector<IMUSample> buf_;
    size_t reserve_{1024};
};

namespace IMUStore {
    IMUBuffer log;
}

static pid_t spawn_python_bridge(const char* py, const char* script) {
    pid_t pid = fork();
    if (pid == 0) { execl(py, py, "-u", script, (char*)nullptr); perror("execl"); _exit(127); }
    if (pid < 0) perror("fork");
    return pid;
}

static void stop_python_bridge(pid_t pid, int grace_ms=300) {
    if (pid <= 0) return;
    kill(pid, SIGTERM);
    for (int i=0; i<grace_ms/10; ++i) {
        int st=0; pid_t r=waitpid(pid, &st, WNOHANG);
        if (r == pid) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    kill(pid, SIGKILL); waitpid(pid, nullptr, WNOHANG);
}

bool announced = false;

void imuThread() {
    #pragma pack(push,1)
    struct ImuHeader { uint8_t type, ver; uint16_t rsv; uint64_t t_ns; uint32_t seq; };
    struct ImuPacket { ImuHeader h; float ax, ay, az, qi, qj, qk, qr, gz; };
    #pragma pack(pop)

    constexpr int IMU_PORT = 5556;

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { perror("imu socket"); return; }

    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    int rcvbuf = 1<<20;
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port   = htons(IMU_PORT);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(fd, (sockaddr*)&a, sizeof(a)) < 0) {
        perror("imu bind");
        close(fd);
        return;
    }

    fcntl(fd, F_SETFL, O_NONBLOCK);
    IMUStore::log.reserve(4096);

    while (running.load(std::memory_order_relaxed)) {
        for (;;) {
            ImuPacket p{};
            ssize_t n = recv(fd, &p, sizeof(p), 0);
            if (n < 0) {
                break;
            }
            if (n == (ssize_t)sizeof(p) && p.h.type == 3 && p.h.ver == 2) {
                IMUSample s{
                    now_ns(),
                    p.h.seq,
                    p.ax, p.ay, p.az,
                    p.qi, p.qj, p.qk, p.qr,
                    p.gz
                };
                IMUStore::log.push(s);
                if (!announced) { std::cerr << "[imu] streaming (ver=2)\n"; announced = true; }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    close(fd);
}

bool direction = true; // true - clockwise

sl_lidar_response_measurement_node_hq_t nodes[8192];

cv::Mat m_map(620, 620, CV_8UC1);
uchar* map = m_map.data;

cv::Mat frame(240, 640, CV_8UC3);
cv::Mat image(240, 640, CV_8UC3);
cv::Mat hsv(240, 640, CV_8UC3);

cv::Mat red_mask(240, 640, CV_8UC1);
cv::Mat green_mask(240, 640, CV_8UC1);
cv::Mat pink_mask(240, 640, CV_8UC1);
cv::Mat wall_mask(240, 640, CV_8UC1);
uchar* red_mask_p = red_mask.data;
uchar* green_mask_p = green_mask.data;
uchar* pink_mask_p = pink_mask.data;
uchar* wall_mask_p = wall_mask.data;

cv::Mat m_draw(620, 620, CV_8UC1);
uchar* draw = m_draw.data;

int missMatchCounter = 0, ChangeZoneThreshould = 2, zone = 0;
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

void process_hsv(uchar*& hsv_p, uchar*& red_p, uchar*& green_p, uchar*& pink_p, uchar*& wall_p) {
    int hue, sat, val;
    for (int p = 0; p < 240 * 640; p++) {
        red_p[p] = 0;
        green_p[p] = 0;
        pink_p[p] = 0;
        wall_p[p] = 0;

        hue = hsv_p[p * 3    ];
        sat = hsv_p[p * 3 + 1];
        val = hsv_p[p * 3 + 2];

        if (sat > 120 && val > 50 && val < 254) {
            if (hue < 15 || 175 < hue) {
                red_p[p] = 255;
            }
        }

        if (sat > 120 && val > 50 && val < 254) {
            if (45 < hue && hue < 90) {
                green_p[p] = 255;
            }
        }

        if (sat > 120 && val > 60 && val < 254) {
            if (135 < hue && hue <= 175) {
                pink_p[p] = 255;
            }
        }

        if (val < 60) {
            wall_p[p] = 255;
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

    // since lines always intersect, det != 0
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

    if (B1 == 0) return -1;

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

	    long long sumAt90 = sum[90+15] - sum[90-16];
	    long long sumAt270 = sum[270+15] - sum[270-16];
	    if (sumAt90 >= sumAt270) {
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

void rideAtAngle(double A, double time) {
    double beginTime = now_ns();
    double err = 0, errold = 0;

    auto imu_batch = IMUStore::log.take_and_clear();
    while (running.load(std::memory_order_relaxed)) {
	auto imu_batch = IMUStore::log.take_and_clear();

	if (!imu_batch.empty()) {
	    // y back, x right
	    static uint64_t prev_ns = 0;
	    double axl, ayl, ax, ay;
	    for (const auto& s : imu_batch) {
		double dt = (prev_ns == 0) ? 0.0 : (double)(s.t_ns - prev_ns) * 1e-9;
		if (dt <= 0.0) dt = 0.000001;
		dt = (dt > 0.02) ? 0.02 : dt;
		prev_ns = s.t_ns;

		float w = s.qr, x = s.qi, y = s.qj, z = s.qk;
		float n = std::sqrt(w*w + x*x + y*y + z*z);
		if (n > 0.0f) { w /= n; x /= n; y /= n; z /= n; }
		double a = std::atan2(float(2.0f * (w*z + x*y)), float(1.0f - 2.0f * (y*y + z*z)));
		a = a + 2 * M_PI;
		if (a > 2 * M_PI) a -= 2 * M_PI;

		double dir = 0.48;
		err = a - A;
		dir = 0.48 + err * 0.25 + (err - errold) * 5;
		dir = std::min(std::max(dir, 0.42), 0.54);
		servo(dir);
		errold = err;
	    }
	}

	if (now_ns() - beginTime >= time) break;

	if (is_button_down()) buttonSum++;
	else buttonSum--;
	if (buttonSum < 0) buttonSum = 0;
	if (buttonSum > 100) running = false;
    }
}

void rideUntilAngle(double A, bool dir, double time) {
    double beginTime = now_ns();
    bool endThis = false;

    auto imu_batch = IMUStore::log.take_and_clear();

    while (running.load(std::memory_order_relaxed)) {
	auto imu_batch = IMUStore::log.take_and_clear();

	if (!imu_batch.empty()) {
	    // y back, x right
	    static uint64_t prev_ns = 0;
	    double axl, ayl, ax, ay;
	    for (const auto& s : imu_batch) {
		double dt = (prev_ns == 0) ? 0.0 : (double)(s.t_ns - prev_ns) * 1e-9;
		if (dt <= 0.0) dt = 0.000001;
		dt = (dt > 0.02) ? 0.02 : dt;
		prev_ns = s.t_ns;

		float w = s.qr, x = s.qi, y = s.qj, z = s.qk;
		float n = std::sqrt(w*w + x*x + y*y + z*z);
		if (n > 0.0f) { w /= n; x /= n; y /= n; z /= n; }
		double a = std::atan2(float(2.0f * (w*z + x*y)), float(1.0f - 2.0f * (y*y + z*z)));
		a = a + 2 * M_PI;
		while (a < 0) a += 2 * M_PI;
		while (a > 2 * M_PI) a -= 2 * M_PI;
		if (dir) {
		    if (a >= A) {
			endThis = true;
			break;
		    }
		} else {
		    if (a <= A) {
			endThis = true;
			break;
		    }
		}
	    }
	}

	if (endThis) break;

	if (time != -1 && now_ns() - beginTime >= time) break;

	if (is_button_down()) buttonSum++;
	else buttonSum--;
	if (buttonSum < 0) buttonSum = 0;
	if (buttonSum > 100) running = false;
    }
}

void wait(double time) {
    double startTime = now_ns();
    while (now_ns() - startTime < time) {}
}


void main_cycle() {
    double loop_t0 = now_ns();
    double errold = 0;
    bool speedStatus = true;
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
		y = std::min(std::max(y+310, 0), 619);

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
		if (lines[i].first < 30) continue;

		long long len_current;
		if (direction) {
		    len_current = RayToLineIntersectLength(lines[i].second, cv::Point2d(310, 310), 0.0);
		} else {
		    len_current = RayToLineIntersectLength(lines[i].second, cv::Point2d(310, 310), M_PI);
		}

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
		    std::cout << "Now in zone " << zone << " at " << (now_ns() - loop_t0) * 1e-9 << "\n";
		}
	    }

	    if (zone >= 24) break;
	}

	if (frame_id != frame_id_old) {
	    process_raw_frame(raw_frame->data, frame.data);
	    grayWorldWB(frame, image);
	    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
	    process_hsv(hsv.data, red_mask.data, green_mask.data, pink_mask.data, wall_mask.data);

	    double l = 0, r = 0, m = 0;
	    for (int i = 0; i < 160; i++) {
		for (int j = 0; j < 100; j++) {
		    if (wall_mask_p[640 * i + j] == 255) {
			l += 1;
		    }
		}
	    }
	    l /= 160 * 100;

	    for (int i = 0; i < 160; i++) {
		for (int j = 540; j < 640; j++) {
		    if (wall_mask_p[640 * i + j] == 255) {
			r += 1;
		    }
		}
	    }
	    r /= 160 * 100;

	    for (int i = 0; i < 160; i++) {
		for (int j = 100; j < 540; j++) {
		    if (wall_mask_p[640 * i + j] == 255) {
			m += 1;
		    }
		}
	    }
	    m /= 160 * 440;

	    double dir = 0.48;

	    if (direction) dir = 0.49;
	    else dir = 0.47;
	    std::vector<std::vector<cv::Point> > red_contours;
	    std::vector<std::vector<cv::Point> > green_contours;
	    std::vector<std::vector<cv::Point> > pink_contours;

	    cv::findContours(red_mask, red_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	    cv::findContours(green_mask, green_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	    cv::findContours(pink_mask, pink_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	    std::vector<std::pair<double, std::pair<cv::Rect, int> > > target;
	    double biggestArea = 0;
	    for (size_t i = 0; i < red_contours.size(); i++) {
		double area = cv::contourArea(red_contours[i]);
		cv::Rect bbox = cv::boundingRect(red_contours[i]);
		if (area < 50) continue;
		biggestArea = std::max(biggestArea, area);
		if (bbox.width < bbox.height) target.push_back({area, {bbox, 1}});
	    }

	    for (size_t i = 0; i < green_contours.size(); i++) {
		double area = cv::contourArea(green_contours[i]);
		cv::Rect bbox = cv::boundingRect(green_contours[i]);
		if (area < 50) continue;
		biggestArea = std::max(biggestArea, area);
		if (bbox.width < bbox.height) target.push_back({area, {bbox, 0}});
	    }
	    /*
	    if (speedStatus && biggestArea > 1000) {
		speedStatus = false;
		motor(0.45);
	    }

	    if (!speedStatus && biggestArea <= 1000) {
		speedStatus = true;
		motor(0.5);
	    }//*/

	    std::sort(target.begin(), target.end(), [](const auto &a, const auto &b) { return a.first > b.first; });

	    if (target.size() != 0) {
		double err = 0;
		if (target[0].second.second == 0) {
		    err = (double(640 - target[0].second.first.x) * -0.5 + 170.0) - double(target[0].second.first.y + target[0].second.first.height);
		} else {
		    err = double(target[0].second.first.y + target[0].second.first.height) - (double(target[0].second.first.x + target[0].second.first.width) * -0.5 + 170.0);
		}
		dir = 0.48 + err * 0.004;
		dir = std::min(std::max(dir, 0.42), 0.54);
		errold = err;
	    }

	    for (size_t i = 0; i < pink_contours.size(); i++) {
		double area = cv::contourArea(pink_contours[i]);
		cv::Rect bbox = cv::boundingRect(pink_contours[i]);
		if (dir != 0.48) { continue;
		    if (area < 2000) continue;
		} else {
		    if (area < 500) continue;
		}
		double err = 0;
		if (direction) {
		    err = double(bbox.y + bbox.height) - (double(bbox.x + bbox.width) * -0.46 + 170.0);
		} else {
		    err = (double(640 - bbox.x) * -0.46 + 170.0) - double(bbox.y + bbox.height);
		}
		double dir2 = 0.48 + err * 0.005;
		dir2 = std::min(std::max(dir2, 0.42), 0.54);
		if (direction) {
		    dir = std::max(dir, dir2);
		} else {
		    dir = std::min(dir, dir2);
		}//*/

	    }

	    if (direction) {
		if (l > 0.75) dir = 0.56;
		else if (m > 0.4 && r < 0.85) dir = 0.56;
		else if (r > 0.75) dir = 0.4;
	    } else {
		if (m > 0.45) {
		    if (l > r) dir = 0.56;
		    else dir = 0.4;
		} else {
		    if (r > 0.7) dir = 0.4;
		    else if (l > 0.7) dir = 0.56;
		}
	    }

	    //std::cout << dir << "\n";
	    servo(dir);
	}

	scan_id_old = scan_id;
	frame_id_old = frame_id;

	if (now_ns() - loop_t0 >= 180e9) running = false;

	if (is_button_down()) buttonSum++;
	else buttonSum--;
	if (buttonSum < 0) buttonSum = 0;
	if (buttonSum > 100) running = false;
    }
}

void rotate() {
    double err = 0, errold = 0;
    double bar = 0.7;
    double loop_t0 = now_ns();
    while (running.load(std::memory_order_relaxed)) { //break;
	const cv::Mat* raw_frame = CameraStore::get_latest(frame_id);

	if (frame_id != frame_id_old) {
	    process_raw_frame(raw_frame->data, frame.data);
	    grayWorldWB(frame, image);
	    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
	    process_hsv(hsv.data, red_mask.data, green_mask.data, pink_mask.data, wall_mask.data);

	    double l = 0, r = 0, m = 0;
	    for (int i = 0; i < 160; i++) {
		for (int j = 0; j < 100; j++) {
		    if (wall_mask_p[640 * i + j] == 255) {
			l += 1;
		    }
		}
	    }
	    l /= 160 * 100;

	    for (int i = 0; i < 160; i++) {
		for (int j = 540; j < 640; j++) {
		    if (wall_mask_p[640 * i + j] == 255) {
			r += 1;
		    }
		}
	    }
	    r /= 160 * 100;

	    for (int i = 0; i < 160; i++) {
		for (int j = 100; j < 540; j++) {
		    if (wall_mask_p[640 * i + j] == 255) {
			m += 1;
		    }
		}
	    }
	    m /= 160 * 440;
	    double dir = 0.48;
	    err = l - bar;
	    dir = 0.48 + err * 1 + (err - errold) * 5;
	    dir = std::min(std::max(dir, 0.44), 0.52);
	    if (m > 0.4) {
		break;
	    }
	    servo(dir);
	    errold = err;
	}

	frame_id_old = frame_id;

	if (now_ns() - loop_t0 >= 180e9) running = false;

	if (is_button_down()) buttonSum++;
	else buttonSum--;
	if (buttonSum < 0) buttonSum = 0;
	if (buttonSum > 100) running = false;
    }

    motor(-0.3);
    wait(100e6);
    motor(0);
    servo(0.58);
    wait(500);
    motor(0.5);
    rideUntilAngle(M_PI - (90.0 / 180.0 * M_PI), false, -1);
    motor(0);
}

void park() {
    double x = 3.0;

    motor(0);
    servo(0.4);
    wait(200e6);
    motor(0.45);
    rideUntilAngle(M_PI - ((-40.0 + x) / 180.0 * M_PI), true, -1);
    motor(-0.3);
    wait(200e6);

    motor(0);
    servo(0.6);
    wait(500e6);
    motor(0.45);
    rideUntilAngle(M_PI - ((60.0 + x) / 180.0 * M_PI), false, -1);
    motor(-0.3);
    wait(100e6);

    motor(0);
    servo(0.4);
    wait(500e6);
    motor(0.5);
    rideUntilAngle(M_PI - ((31.0 + x) / 180.0 * M_PI), true, -1);
    motor(-0.3);
    wait(100e6);
    motor(0);

    motor(0);
    servo(0.6);
    wait(500e6);
    motor(-0.52);
    rideUntilAngle(M_PI - ((5.0 + x) / 180.0 * M_PI), true, -1);
    wait(100e6);
    motor(0.3);
    wait(100e6);
    motor(0);
}

void unpark() {
    if (direction) {
	servo(0.58);
	wait(500e6);
	motor(0.5);
	rideUntilAngle(M_PI - (15.0 / 180.0 * M_PI), false, -1);
	motor(-0.2);
	wait(100e6);
	motor(0);
	servo(0.38);
	wait(500e6);
	motor(-0.45);
	rideUntilAngle(M_PI - (37.0 / 180.0 * M_PI), false, 500e6);
	motor(0.2);
	wait(100e6);
	motor(0);
	servo(0.58);
	wait(500e6);
	motor(0.5);
	rideUntilAngle(M_PI - (60.0 / 180.0 * M_PI), false, 500e6);
	servo(0.47);
    } else {
	servo(0.38);
	wait(500e6);
	motor(0.5);
	rideUntilAngle(M_PI - (-15.0 / 180.0 * M_PI), true, -1);
	motor(-0.2);
	wait(100e6);
	motor(0);
	servo(0.58);
	wait(500e6);
	motor(-0.45);
	rideUntilAngle(M_PI - (-37.0 / 180.0 * M_PI), true, 500e6);
	motor(0.2);
	wait(100e6);
	motor(0);
	servo(0.38);
	wait(500e6);
	motor(0.5);
	rideUntilAngle(M_PI - (-60.0 / 180.0 * M_PI), true, 500e6);
	servo(0.49);
    }
}

void goToParking(double bar) {
    double loop_t0 = now_ns();
    double err = 0, errold = 0;
    bool slowdown = false;
    bool last = false;
    while (running.load(std::memory_order_relaxed)) { //break;
	const cv::Mat* raw_frame = CameraStore::get_latest(frame_id);

	if (frame_id != frame_id_old) {
	    process_raw_frame(raw_frame->data, frame.data);
	    grayWorldWB(frame, image);
	    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
	    process_hsv(hsv.data, red_mask.data, green_mask.data, pink_mask.data, wall_mask.data);

	    double l = 0, r = 0, m = 0;
	    for (int i = 0; i < 160; i++) {
		for (int j = 0; j < 100; j++) {
		    if (wall_mask_p[640 * i + j] == 255) {
			l += 1;
		    }
		}
	    }
	    l /= 160 * 100;

	    for (int i = 0; i < 160; i++) {
		for (int j = 540; j < 640; j++) {
		    if (wall_mask_p[640 * i + j] == 255) {
			r += 1;
		    }
		}
	    }
	    r /= 160 * 100;

	    for (int i = 0; i < 160; i++) {
		for (int j = 100; j < 540; j++) {
		    if (wall_mask_p[640 * i + j] == 255) {
			m += 1;
		    }
		}
	    }
	    m /= 160 * 440;
	    std::vector<std::vector<cv::Point> > pink_contours;
	    cv::findContours(pink_mask, pink_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	    double area = 0;
	    for (size_t i = 0; i < pink_contours.size(); i++) {
		area = cv::contourArea(pink_contours[i]);
		cv::Rect bbox = cv::boundingRect(pink_contours[i]);
		if (area > 4000 && !slowdown) {
		    slowdown = true;
		    motor(0.42);
		}
		if (area > 13500) break;
	    }
	    if (area > 13500) break;
	    double dir = 0.48;
	    err = bar-r;
	    dir = 0.48 + err * 1 + (err - errold) * 5;
	    dir = std::min(std::max(dir, 0.44), 0.52);
	    if (m > 0.3) {
		dir = 0.4;
	    }
	    servo(dir);
	    errold = err;
	}

	frame_id_old = frame_id;

	if (now_ns() - loop_t0 >= 180e9) running = false;

	if (is_button_down()) buttonSum++;
	else buttonSum--;
	if (buttonSum < 0) buttonSum = 0;
	if (buttonSum > 100) running = false;
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
    "video/x-raw,width=640,height=480,framerate=30/1 ! "
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

    const char* PY     = "/home/peter/bno-venv/bin/python";
    const char* SCRIPT = "/home/peter/wro/bno.py";
    pid_t imu_child = spawn_python_bridge(PY, SCRIPT);
    if (imu_child <= 0) std::cerr << "Failed to start IMU bridge\n";

    buttonSum = 0;
    while (buttonSum < 50) {
	if (is_button_down()) buttonSum++;
	else buttonSum--;
	if (buttonSum < 0) buttonSum = 0;
    }
    buttonSum = 0;

    std::thread imu_thread(imuThread);
    std::thread lidar_thread(scanThread, drv);
    std::thread camera_thread(cameraThread, &cap);

    servo(0.48);

    findDirection();

    while (!announced) {} // waiting for IMU to start

    unpark()

    motor(0.51);
    main_cycle();

    if (direction) rotate();

    goToParking(0.7);

    motor(-0.3);
    wait(100e6);

    park();

    motor(0);
    servo(0.48);

    cv::Point center_point(310, 310);
    cv::circle(m_map, center_point, 5, 200, 2);
    cv::imwrite("lidar.png", m_map);
    cv::imwrite("frame.png", frame);
    cv::imwrite("image.png", image);
    cv::imwrite("red mask.png", red_mask);
    cv::imwrite("green mask.png", green_mask);
    cv::imwrite("pink mask.png", pink_mask);
    cv::imwrite("wall mask.png", wall_mask);


    running = false;
    drv->stop();
    if (lidar_thread.joinable()) lidar_thread.join();
    if (camera_thread.joinable()) camera_thread.join();
    if (imu_thread.joinable()) imu_thread.join();
    drv->setMotorSpeed(0);
    drv->disconnect();
    stop_python_bridge(imu_child);
    delete ch;
    delete drv;

    return 0;
}
