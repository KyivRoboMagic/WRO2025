#!/usr/bin/env python3
import socket, struct, time, errno, signal, os

from adafruit_extended_bus import ExtendedI2C as I2C
try:    from adafruit_bno08x.i2c import BNO08X_I2C
except: from adafruit_bno08x import BNO08X_I2C

# Feature constants (support both older/newer layouts; final fallback numeric is best-effort)
try:
    from adafruit_bno08x import (
        BNO_REPORT_LINEAR_ACCELERATION,
        BNO_REPORT_ROTATION_VECTOR,
        BNO_REPORT_GYROSCOPE,
    )
except Exception:
    try:
        from adafruit_bno08x.features import (
            BNO_REPORT_LINEAR_ACCELERATION,
            BNO_REPORT_ROTATION_VECTOR,
            BNO_REPORT_GYROSCOPE,
        )
    except Exception:
        BNO_REPORT_LINEAR_ACCELERATION = 0x04
        BNO_REPORT_ROTATION_VECTOR     = 0x05
        BNO_REPORT_GYROSCOPE           = 0x08  # library should provide this; numeric is a fallback

# Config
BUS, ADDR = 1, 0x4A
HOST = os.getenv("BNO_UDP_HOST", "127.0.0.1")
PORT = int(os.getenv("BNO_UDP_PORT", "5556"))
DEST = (HOST, PORT)
MAXERRS = 5
SLEEP_S = 0.0

STOP = False
def _sig(_n, _f):
    global STOP; STOP = True
signal.signal(signal.SIGINT, _sig)
signal.signal(signal.SIGTERM, _sig)

# Header + Body (VER=2, adds gz)
TYPE_COMBINED, VER = 3, 2
HDR  = struct.Struct("<BBH Q I")     # type, ver, rsv, t_ns, seq  (16 bytes)
BODY = struct.Struct("<fff ffff f")  # ax,ay,az, qi,qj,qk,qr, gz  (32 bytes) => total 48
BUF  = bytearray(HDR.size + BODY.size)

def enable_features(bno):
    for feat in (BNO_REPORT_LINEAR_ACCELERATION, BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE):
        try:
            bno.enable_feature(feat, report_interval_us=2500)
        except TypeError:
            bno.enable_feature(feat)

def open_sensor():
    i2c = I2C(BUS)
    bno = BNO08X_I2C(i2c, address=ADDR)
    try:
        if hasattr(bno, "soft_reset"):
            bno.soft_reset(); time.sleep(0.002)
    except Exception:
        pass
    enable_features(bno)
    # brief drain
    t_end = time.time() + 0.05
    while time.time() < t_end:
        try:
            _ = bno.linear_acceleration; _ = bno.quaternion
            _ = getattr(bno, "gyro", getattr(bno, "gyroscope", None))
        except Exception:
            pass
    return i2c, bno

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try: sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1<<20)
    except Exception: pass

    seq = 0
    t0_ns = time.monotonic_ns()

    while not STOP:
        try:
            i2c, bno = open_sensor()
            err = 0
            last_gz = 0.0
            while not STOP:
                try:
                    la = bno.linear_acceleration
                    q  = bno.quaternion
                    g  = getattr(bno, "gyro", getattr(bno, "gyroscope", None))
                    gz = (g[2] if g else last_gz)
                    if la and q:
                        seq += 1
                        t_ns = time.monotonic_ns() - t0_ns
                        HDR.pack_into(BUF, 0, TYPE_COMBINED, VER, 0, t_ns, seq)
                        BODY.pack_into(BUF, HDR.size, la[0], la[1], la[2], q[0], q[1], q[2], q[3], gz)
                        sock.sendto(BUF, DEST)
                        last_gz = gz
                        err = 0
                    if SLEEP_S: time.sleep(SLEEP_S)
                except (KeyError, IndexError, OSError):
                    err += 1
                    time.sleep(0.001)
                    if err >= MAXERRS:
                        try:
                            if hasattr(bno, "soft_reset"):
                                bno.soft_reset(); time.sleep(0.002)
                            enable_features(bno)
                        except Exception:
                            try: i2c.deinit()
                            except Exception: pass
                            break
                except Exception:
                    time.sleep(0.001)

            try:
                if hasattr(bno, "soft_reset"): bno.soft_reset()
            except Exception: pass
            try: i2c.deinit()
            except Exception: pass
            if not STOP: time.sleep(0.02)

        except Exception:
            time.sleep(0.1)

if __name__ == "__main__":
    main()
