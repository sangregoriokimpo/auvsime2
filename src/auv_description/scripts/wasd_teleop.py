#!/usr/bin/env python3
import sys, termios, tty, select, threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

HELP = """\
Controls (body frame):
  Translation  : W/S (+/-X), A/D (+/-Y), E/Q (+/-Z)
  Rotation     : I/K (+/-Pitch), J/L (+/-Yaw), U/O (+/-Roll)
  Space        : zero all (force & torque)

Parameters (ros2 run ... --ros-args -p name:=value):
  force_topic    (string) default: /auve1/force_body
  torque_topic   (string) default: /auve1/torque_body
  force          (double) default: 100.0
  torque         (double) default: 50.0
  rate_hz        (double) default: 60.0
  decay          (double) default: 1.0    # 1.0 = no decay
  torque_decay   (double) default: 1.0    # 1.0 = no decay
"""

def _getch(timeout=0.0):
    """Non-blocking single-char read with timeout (seconds). Returns None if no key."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

class WasdTeleop(Node):
    """
    Publishes body-frame wrench as geometry_msgs/Vector3 on two topics:
      - force_topic  (Vector3 force in body frame)
      - torque_topic (Vector3 torque in body frame)
    Works even if no one subscribes to torque (legacy force-only plugin).
    """
    def __init__(self):
        super().__init__('wasd_teleop')

        # ---- Parameters ----
        self.declare_parameter('force_topic', '/auve1/force_body')     # absolute default (no accidental "~")
        self.declare_parameter('torque_topic', '/auve1/torque_body')   # absolute default
        self.declare_parameter('force', 100.0)
        self.declare_parameter('torque', 50.0)
        self.declare_parameter('rate_hz', 60.0)
        self.declare_parameter('decay', 1.0)
        self.declare_parameter('torque_decay', 1.0)

        self.force_topic  = self.get_parameter('force_topic').get_parameter_value().string_value
        self.torque_topic = self.get_parameter('torque_topic').get_parameter_value().string_value
        self.force_mag    = float(self.get_parameter('force').value)
        self.torque_mag   = float(self.get_parameter('torque').value)
        self.rate_hz      = max(1.0, float(self.get_parameter('rate_hz').value))
        self.decay        = float(self.get_parameter('decay').value)
        self.torque_decay = float(self.get_parameter('torque_decay').value)

        qos = 10
        self.pub_force  = self.create_publisher(Vector3, self.force_topic, qos)
        self.pub_torque = self.create_publisher(Vector3, self.torque_topic, qos)

        self.get_logger().info(f'Publishing force  Vector3 on "{self.pub_force.topic_name}"')
        self.get_logger().info(f'Publishing torque Vector3 on "{self.pub_torque.topic_name}"')
        self.get_logger().info(HELP)

        # Current command values (body-frame)
        self.f = Vector3()
        self.t = Vector3()

        # Timer for periodic publish (and decay)
        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self._on_timer)

        # For clean shutdown of the keyboard thread (if you add one later)
        self._stop_event = threading.Event()

    # ---- Periodic tick ----
    def _on_timer(self):
        # Exponential decay toward zero (if < 1.0)
        self.f.x *= self.decay
        self.f.y *= self.decay
        self.f.z *= self.decay

        self.t.x *= self.torque_decay
        self.t.y *= self.torque_decay
        self.t.z *= self.torque_decay

        # Publish keep-alive so plugin can hold for <hold_ms>
        self.pub_force.publish(self.f)
        self.pub_torque.publish(self.t)

    # ---- Key handler ----
    def handle_key(self, ch: str):
        ch = ch.lower()
        updated = False

        # Translation
        if ch == 'w':   self.f.x =  self.force_mag; self.f.y = 0.0;           updated = True
        elif ch == 's': self.f.x = -self.force_mag; self.f.y = 0.0;           updated = True
        elif ch == 'a': self.f.y =  self.force_mag; self.f.x = 0.0;           updated = True
        elif ch == 'd': self.f.y = -self.force_mag; self.f.x = 0.0;           updated = True
        elif ch == 'e': self.f.z =  self.force_mag;                           updated = True
        elif ch == 'q': self.f.z = -self.force_mag;                           updated = True

        # Rotation (body)
        elif ch == 'i': self.t.y =  self.torque_mag;                           updated = True  # +Pitch
        elif ch == 'k': self.t.y = -self.torque_mag;                           updated = True  # -Pitch
        elif ch == 'j': self.t.z =  self.torque_mag;                           updated = True  # +Yaw
        elif ch == 'l': self.t.z = -self.torque_mag;                           updated = True  # -Yaw
        elif ch == 'u': self.t.x =  self.torque_mag;                           updated = True  # +Roll
        elif ch == 'o': self.t.x = -self.torque_mag;                           updated = True  # -Roll

        # Zero all
        elif ch == ' ':
            self.f.x = self.f.y = self.f.z = 0.0
            self.t.x = self.t.y = self.t.z = 0.0
            updated = True

        if not updated:
            return

        # Publish immediately for instant response
        self.pub_force.publish(self.f)
        self.pub_torque.publish(self.t)

        self.get_logger().info(
            f"Key '{ch}' "
            f"=> F[x,y,z]=({self.f.x:.1f},{self.f.y:.1f},{self.f.z:.1f})  "
            f"T[roll,pitch,yaw]=({self.t.x:.1f},{self.t.y:.1f},{self.t.z:.1f})"
        )

    def stop(self):
        self._stop_event.set()

def main():
    rclpy.init()
    node = WasdTeleop()

    stdin_is_tty = sys.stdin.isatty()
    if stdin_is_tty:
        print(HELP)

    try:
        while rclpy.ok():
            # Process timers/callbacks
            rclpy.spin_once(node, timeout_sec=0.02)

            # Poll keyboard (non-blocking)
            if stdin_is_tty:
                ch = _getch(0.0)
                if ch:
                    if ch == '\x03':  # Ctrl-C
                        break
                    node.handle_key(ch)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.get_logger().info("Shutting down wasd_teleop.")
        except Exception:
            pass
        node.stop()
        if rclpy.ok():  # avoid "rcl_shutdown already called"
            rclpy.shutdown()

if __name__ == '__main__':
    main()
