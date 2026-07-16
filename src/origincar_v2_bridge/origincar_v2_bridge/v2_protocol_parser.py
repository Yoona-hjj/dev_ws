"""
V2 Protocol Parser for OriginCar STM32 (Ackermann + RDK X5)
============================================================
Frame: 42 bytes, big-endian, CRC16-CCITT-FALSE
Header: 0xAA 0x55, Tail: 0x0D
100 Hz uplink from STM32

Usage:
    parser = V2Parser()
    for frame in parser.feed(serial_chunk):
        # frame is a dict with all decoded fields
"""

import struct
import math
from dataclasses import dataclass, field
from typing import Generator, Dict, Any, Optional

# ============ Constants ============
V2_PACKET_SIZE = 42
V2_HEADER = b"\xAA\x55"
V2_VERSION = 0x02
V2_TAIL = 0x0D

# Unit conversions (raw → SI)
G_ACC = 9.80665
GYRO_FS_DPS = 500
ACCEL_FS_G = 2
GYRO_LSB_TO_RAD_S = (GYRO_FS_DPS * math.pi / 180.0) / 32768.0  # ≈ 2.6643e-4
ACC_LSB_TO_M_S2 = (ACCEL_FS_G * G_ACC) / 32768.0                # ≈ 5.9855e-4

# Ackermann kinematics
WHEEL_DIAMETER = 0.065           # m
WHEEL_PERIMETER = WHEEL_DIAMETER * math.pi   # ≈ 0.2042 m
ENCODER_PRECISION = 4 * 13 * 30  # = 1560 counts/rev
AKM_WHEELBASE = 0.144            # m (axle spacing)
AKM_TRACK_REAR = 0.162           # m (rear wheel spacing)


def crc16_ccitt_false(buf: bytes) -> int:
    """CRC16-CCITT-FALSE: poly=0x1021, init=0xFFFF, no reflect, no xorout"""
    crc = 0xFFFF
    for b in buf:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


@dataclass
class V2Frame:
    """Decoded V2 frame with SI units and ROS coordinate convention"""
    # Raw protocol fields
    seq: int = 0
    flag: int = 0
    timestamp_us: int = 0
    enc_total_A: int = 0   # left rear cumulative
    enc_total_B: int = 0   # right rear cumulative
    enc_total_C: int = 0   # (always 0 for Ackermann)
    enc_total_D: int = 0   # (always 0 for Ackermann)
    voltage_mV: int = 0

    # Flag bit fields
    flag_stop: bool = False
    imu_calibrated: bool = False
    robot_static: bool = False
    car_mode: int = 0

    # SI converted IMU (ROS convention: x-forward, y-left, z-up)
    gyro_x: float = 0.0    # rad/s
    gyro_y: float = 0.0    # rad/s
    gyro_z: float = 0.0    # rad/s
    acc_x: float = 0.0     # m/s²
    acc_y: float = 0.0     # m/s²
    acc_z: float = 0.0     # m/s²

    # Computed odometry
    vx_body: float = 0.0   # m/s (forward)
    vy_body: float = 0.0   # m/s (lateral, always 0 for Ackermann)
    omega_enc: float = 0.0 # rad/s (from encoder differential)
    omega_imu: float = 0.0 # rad/s (from IMU gyro_z)
    dt: float = 0.0        # seconds since last frame

    # Voltage
    voltage_V: float = 0.0


class V2Parser:
    """
    Stateful V2 protocol parser with frame synchronization.
    Feed serial bytes via feed(), yields decoded V2Frame objects.
    """

    def __init__(self):
        self._buf = bytearray()
        self._last_ts_us: Optional[int] = None
        self._last_enc_A: Optional[int] = None
        self._last_enc_B: Optional[int] = None
        self._last_seq: Optional[int] = None

        # Statistics
        self.ok_count = 0
        self.crc_err_count = 0
        self.seq_lost_count = 0

    def feed(self, data: bytes) -> Generator[V2Frame, None, None]:
        """Feed raw serial bytes, yield decoded frames."""
        self._buf.extend(data)

        while len(self._buf) >= V2_PACKET_SIZE:
            # Scan for header
            idx = self._buf.find(V2_HEADER)
            if idx < 0:
                # No header found, keep last byte (could be partial header)
                self._buf = self._buf[-1:]
                return
            if idx > 0:
                # Discard bytes before header
                self._buf = self._buf[idx:]

            if len(self._buf) < V2_PACKET_SIZE:
                return

            packet = bytes(self._buf[:V2_PACKET_SIZE])

            # Validate version and tail
            if packet[2] != V2_VERSION or packet[41] != V2_TAIL:
                self._buf = self._buf[1:]
                continue

            # Validate CRC16
            crc_calc = crc16_ccitt_false(packet[0:39])
            crc_rx = struct.unpack(">H", packet[39:41])[0]
            if crc_calc != crc_rx:
                self.crc_err_count += 1
                self._buf = self._buf[1:]
                continue

            # Valid frame - consume it
            self._buf = self._buf[V2_PACKET_SIZE:]
            self.ok_count += 1

            frame = self._decode(packet)
            if frame is not None:
                yield frame

    def _decode(self, packet: bytes) -> Optional[V2Frame]:
        """Decode a validated 42-byte packet into a V2Frame."""
        f = V2Frame()

        # Parse fields
        f.seq = packet[3]
        f.flag = packet[4]
        f.timestamp_us = struct.unpack(">I", packet[5:9])[0]
        f.enc_total_A, f.enc_total_B, f.enc_total_C, f.enc_total_D = \
            struct.unpack(">iiii", packet[9:25])
        gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw = \
            struct.unpack(">hhhhhh", packet[25:37])
        f.voltage_mV = struct.unpack(">H", packet[37:39])[0]

        # Flag bits
        f.flag_stop = bool(f.flag & 0x01)
        f.imu_calibrated = bool(f.flag & 0x02)
        f.robot_static = bool(f.flag & 0x04)
        f.car_mode = (f.flag >> 3) & 0x07

        # IMU → ROS coordinate transform (REP-103: x-forward, y-left, z-up)
        # MPU6050 chip axes → ROS body: ROS_x = gy, ROS_y = -gx, ROS_z = gz
        f.gyro_x = gy_raw * GYRO_LSB_TO_RAD_S
        f.gyro_y = -gx_raw * GYRO_LSB_TO_RAD_S
        f.gyro_z = gz_raw * GYRO_LSB_TO_RAD_S
        f.acc_x = ay_raw * ACC_LSB_TO_M_S2
        f.acc_y = -ax_raw * ACC_LSB_TO_M_S2
        f.acc_z = az_raw * ACC_LSB_TO_M_S2

        # Voltage
        f.voltage_V = f.voltage_mV / 1000.0

        # Sequence tracking
        if self._last_seq is not None:
            expected = (self._last_seq + 1) & 0xFF
            if f.seq != expected:
                lost = (f.seq - expected) & 0xFF
                self.seq_lost_count += lost
        self._last_seq = f.seq

        # Compute dt from DWT timestamp (handles uint32 wraparound)
        if self._last_ts_us is not None:
            dt_us = (f.timestamp_us - self._last_ts_us) & 0xFFFFFFFF
            f.dt = dt_us / 1e6
            # Sanity: reject unreasonable dt (> 1s or < 1ms).
            # dt=0 skips the velocity computation below; using a fake 10ms
            # here would divide a large encoder delta by a tiny dt and
            # produce a huge vx spike after a serial stall/reconnect.
            if f.dt > 1.0 or f.dt < 0.001:
                f.dt = 0.0
        else:
            f.dt = 0.01  # first frame, assume 10ms
        self._last_ts_us = f.timestamp_us

        # Compute wheel velocities and body velocity
        if self._last_enc_A is not None and f.dt > 0:
            delta_A = f.enc_total_A - self._last_enc_A
            delta_B = f.enc_total_B - self._last_enc_B
            v_left = delta_A * WHEEL_PERIMETER / ENCODER_PRECISION / f.dt
            v_right = delta_B * WHEEL_PERIMETER / ENCODER_PRECISION / f.dt
            f.vx_body = 0.5 * (v_left + v_right)
            f.vy_body = 0.0  # Ackermann non-holonomic constraint
            f.omega_enc = (v_right - v_left) / AKM_TRACK_REAR
        else:
            f.vx_body = 0.0
            f.vy_body = 0.0
            f.omega_enc = 0.0

        f.omega_imu = f.gyro_z  # Prefer IMU for yaw rate

        self._last_enc_A = f.enc_total_A
        self._last_enc_B = f.enc_total_B

        return f

    def reset(self):
        """Reset parser state (e.g., on reconnection)."""
        self._buf.clear()
        self._last_ts_us = None
        self._last_enc_A = None
        self._last_enc_B = None
        self._last_seq = None

    @property
    def stats(self) -> Dict[str, int]:
        return {
            'ok_count': self.ok_count,
            'crc_err_count': self.crc_err_count,
            'seq_lost_count': self.seq_lost_count,
        }


# ============ CLI test mode ============
if __name__ == "__main__":
    import sys
    import serial

    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print(f"Opening {port} @ {baud}...")
    ser = serial.Serial(port, baud, timeout=0.1)
    parser = V2Parser()
    count = 0

    try:
        while True:
            chunk = ser.read(256)
            if not chunk:
                continue
            for frame in parser.feed(chunk):
                count += 1
                if count % 100 == 0:
                    print(f"[{count}] seq={frame.seq} dt={frame.dt*1000:.1f}ms "
                          f"vx={frame.vx_body:.3f} ωz_imu={frame.omega_imu:.3f} "
                          f"ωz_enc={frame.omega_enc:.3f} "
                          f"acc=({frame.acc_x:.2f},{frame.acc_y:.2f},{frame.acc_z:.2f}) "
                          f"cal={frame.imu_calibrated} static={frame.robot_static} "
                          f"V={frame.voltage_V:.2f} "
                          f"| ok={parser.ok_count} crc_err={parser.crc_err_count} "
                          f"seq_lost={parser.seq_lost_count}")
    except KeyboardInterrupt:
        print(f"\nFinal stats: {parser.stats}")
        ser.close()
