"""
V2 Serial Node for OriginCar (Ackermann + RDK X5)
===================================================
Reads V2 protocol from STM32 serial, publishes:
  - /imu/data_raw       (sensor_msgs/Imu)
  - /wheel_odom         (nav_msgs/Odometry)
  - /battery_state      (sensor_msgs/BatteryState)
  - /robot_static       (std_msgs/Bool)

Also subscribes to /cmd_vel and forwards as V1 downlink to STM32.
"""

import math
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu, BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

import serial

from .v2_protocol_parser import V2Parser, V2Frame


# V1 downlink command frame (unchanged from original OriginCar)
V1_FRAME_HEADER = 0x7B
V1_FRAME_TAIL = 0x7D
V1_SEND_SIZE = 11


class V2SerialNode(Node):
    def __init__(self):
        super().__init__('v2_serial_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('publish_tf', False)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # Publishers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', qos)
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', qos)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.static_pub = self.create_publisher(Bool, '/robot_static', 10)

        # Subscriber for cmd_vel (downlink to STM32)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos)

        # TF broadcaster (optional)
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Serial port
        self.ser = None
        self.parser = V2Parser()
        self.serial_lock = threading.Lock()

        # Time synchronization: build a monotonic ROS stamp by accumulating the
        # reliable per-frame dt (parser handles uint32 wraparound + clamps bad
        # dt), instead of mapping the STM absolute timestamp which periodically
        # jumps and glitches EKF/Madgwick timing.
        self.stamp_ros = None

        # Odometry integration (for pose, not used by EKF but useful for viz)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        # Open serial
        self._open_serial()

        # Read thread
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

        # Battery publish timer (2 Hz)
        self.last_voltage = 0.0
        self.battery_timer = self.create_timer(0.5, self._publish_battery)

        # Stats timer (10s)
        self.stats_timer = self.create_timer(10.0, self._log_stats)

        self.get_logger().info(
            f'V2 serial node started: {self.port} @ {self.baud}')

    def _open_serial(self):
        """Open serial port with retry."""
        try:
            self.ser = serial.Serial(
                self.port, self.baud,
                timeout=0.05,
                inter_byte_timeout=0.01)
            self.get_logger().info(f'Serial opened: {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open serial {self.port}: {e}')
            self.ser = None

    def _frame_stamp(self, dt: float):
        """Build a monotonic ROS stamp by accumulating the reliable per-frame dt.

        The STM absolute timestamp is unreliable (periodic ~25s jumps), so we
        advance our own anchor by dt each frame. dt is already wraparound-safe
        and range-clamped by the parser. We only hard-resync to wall clock if
        the accumulator drifts too far (serial stall/reconnect), which prevents
        EKF sensor_timeout while avoiding periodic timestamp glitches.
        """
        from rclpy.time import Time

        now = self.get_clock().now()
        if self.stamp_ros is None:
            self.stamp_ros = now
            return now

        # Advance by reliable per-frame dt. dt<=0 means the parser rejected the
        # frame timing (stall/wrap); fall back to nominal 100 Hz spacing.
        step = dt if dt > 0.0 else 0.01
        self.stamp_ros = Time(nanoseconds=self.stamp_ros.nanoseconds + int(step * 1e9))

        # Guard against unbounded divergence from wall clock (crystal drift is
        # negligible; this only triggers after a real stall/reconnect).
        err_ns = now.nanoseconds - self.stamp_ros.nanoseconds
        if abs(err_ns) > 200_000_000:  # 200 ms
            self.get_logger().warn(
                f'Frame stamp resync: err={err_ns / 1e6:.1f} ms')
            self.stamp_ros = now
        return self.stamp_ros

    def _read_loop(self):
        """Background thread: read serial, parse, publish."""
        while self.running:
            if self.ser is None or not self.ser.is_open:
                import time
                time.sleep(1.0)
                self._open_serial()
                continue

            try:
                chunk = self.ser.read(256)
                if not chunk:
                    continue
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial read error: {e}')
                self.ser = None
                continue

            for frame in self.parser.feed(chunk):
                self._publish_frame(frame)

    def _publish_frame(self, f: V2Frame):
        """Publish IMU, wheel odometry, and static flag from a decoded frame."""
        stamp = self._frame_stamp(f.dt).to_msg()

        # --- IMU ---
        # README §5: imu_calibrated==0 时不应让 EKF 信任 IMU
        # 策略: 未校准时膨胀协方差，让 EKF 自动降权
        imu_cov = 2.5e-5 if f.imu_calibrated else 1e6

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.imu_frame_id

        # No orientation from raw IMU (let Madgwick compute it)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 0.0
        imu_msg.orientation_covariance[0] = -1.0  # indicates no orientation

        imu_msg.angular_velocity.x = f.gyro_x
        imu_msg.angular_velocity.y = f.gyro_y
        imu_msg.angular_velocity.z = f.gyro_z
        imu_msg.angular_velocity_covariance[0] = imu_cov
        imu_msg.angular_velocity_covariance[4] = imu_cov
        imu_msg.angular_velocity_covariance[8] = imu_cov

        imu_msg.linear_acceleration.x = f.acc_x
        imu_msg.linear_acceleration.y = f.acc_y
        imu_msg.linear_acceleration.z = f.acc_z
        acc_cov = 2.5e-3 if f.imu_calibrated else 1e6
        imu_msg.linear_acceleration_covariance[0] = acc_cov
        imu_msg.linear_acceleration_covariance[4] = acc_cov
        imu_msg.linear_acceleration_covariance[8] = acc_cov

        try:
            self.imu_pub.publish(imu_msg)
        except Exception:
            return

        # --- Wheel Odometry ---
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id

        # Twist (velocity)
        odom_msg.twist.twist.linear.x = f.vx_body
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = f.omega_imu  # Use IMU for yaw rate

        # Twist covariance (vx, vy, vz, vroll, vpitch, vyaw)
        if f.dt <= 0.0:
            # dt was invalid (serial stall) -> velocity is unknown this frame
            sigma_vx = 1e3
        else:
            sigma_vx = 0.02 if abs(f.omega_enc) < 0.1 else 0.08
        odom_msg.twist.covariance[0] = sigma_vx ** 2     # vx
        odom_msg.twist.covariance[7] = 1e-9              # vy (not measured)
        odom_msg.twist.covariance[14] = 1e-9             # vz
        odom_msg.twist.covariance[21] = 1e9              # vroll
        odom_msg.twist.covariance[28] = 1e9              # vpitch
        odom_msg.twist.covariance[35] = 0.005 ** 2       # vyaw

        # Integrate pose for visualization (EKF will override this)
        if f.dt > 0:
            self.odom_yaw += f.omega_imu * f.dt
            self.odom_x += f.vx_body * math.cos(self.odom_yaw) * f.dt
            self.odom_y += f.vx_body * math.sin(self.odom_yaw) * f.dt

        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.position.z = 0.0

        # Quaternion from yaw
        odom_msg.pose.pose.orientation.z = math.sin(self.odom_yaw / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.odom_yaw / 2.0)

        # Pose covariance (large - EKF should not trust this integrated pose)
        odom_msg.pose.covariance[0] = 1e6
        odom_msg.pose.covariance[7] = 1e6
        odom_msg.pose.covariance[35] = 1e6

        try:
            self.odom_pub.publish(odom_msg)
        except Exception:
            return

        # --- Robot Static flag ---
        static_msg = Bool()
        static_msg.data = f.robot_static
        try:
            self.static_pub.publish(static_msg)
        except Exception:
            return

        # --- Store voltage for battery timer ---
        self.last_voltage = f.voltage_V

    def _publish_battery(self):
        """Publish battery state at low frequency."""
        if self.last_voltage <= 0:
            return
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = self.last_voltage
        msg.present = True
        try:
            self.battery_pub.publish(msg)
        except Exception:
            return

    def _log_stats(self):
        """Log parser statistics periodically."""
        s = self.parser.stats
        self.get_logger().info(
            f'V2 stats: ok={s["ok_count"]} crc_err={s["crc_err_count"]} '
            f'seq_lost={s["seq_lost_count"]}')

    def cmd_vel_callback(self, msg: Twist):
        """Forward cmd_vel to STM32 via V1 downlink protocol (unchanged)."""
        self._send_velocity(msg.linear.x, msg.linear.y, msg.angular.z)

    def _send_velocity(self, vx: float, vy: float, wz: float):
        """Encode and write a single V1 downlink velocity frame to the STM32."""
        if self.ser is None or not self.ser.is_open:
            return

        tx = bytearray(V1_SEND_SIZE)
        tx[0] = V1_FRAME_HEADER
        tx[1] = 0
        tx[2] = 0

        # linear.x → tx[3:4] (big-endian i16, *1000)
        vx_int = int(vx * 1000)
        tx[3] = (vx_int >> 8) & 0xFF
        tx[4] = vx_int & 0xFF

        # linear.y → tx[5:6]
        vy_int = int(vy * 1000)
        tx[5] = (vy_int >> 8) & 0xFF
        tx[6] = vy_int & 0xFF

        # angular.z → tx[7:8]
        wz_int = int(wz * 1000)
        tx[7] = (wz_int >> 8) & 0xFF
        tx[8] = wz_int & 0xFF

        # XOR checksum
        checksum = 0
        for i in range(9):
            checksum ^= tx[i]
        tx[9] = checksum
        tx[10] = V1_FRAME_TAIL

        with self.serial_lock:
            try:
                self.ser.write(tx)
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial write error: {e}')

    def _send_stop(self):
        """Command the STM32 to halt. Sent on shutdown so the chassis doesn't
        keep running the last latched velocity after the node exits."""
        if self.ser is None or not self.ser.is_open:
            return
        import time
        for _ in range(5):
            self._send_velocity(0.0, 0.0, 0.0)
            try:
                self.ser.flush()
            except serial.SerialException:
                pass
            time.sleep(0.02)

    def destroy_node(self):
        self.running = False
        # Stop the wheels BEFORE closing the port, otherwise the firmware keeps
        # driving the last commanded velocity.
        self._send_stop()
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = V2SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
