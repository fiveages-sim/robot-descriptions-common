#!/usr/bin/env python3
"""Merge remote chassis and local body JointState streams into one /joint_states."""

from __future__ import annotations

from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMux(Node):
    """Combine partial JointState messages from two sources by joint name."""

    def __init__(self) -> None:
        super().__init__("joint_state_mux")
        self.declare_parameter("chassis_joint_states_topic", "/chassis/joint_states")
        self.declare_parameter("body_joint_states_topic", "/body/joint_states")
        self.declare_parameter("output_topic", "/joint_states")
        self.declare_parameter("publish_rate_hz", 100.0)

        chassis_topic = self.get_parameter("chassis_joint_states_topic").value
        body_topic = self.get_parameter("body_joint_states_topic").value
        output_topic = self.get_parameter("output_topic").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self._positions: Dict[str, float] = {}
        self._velocities: Dict[str, float] = {}
        self._efforts: Dict[str, float] = {}
        self._joint_order: List[str] = []
        self._last_header = None

        qos = rclpy.qos.QoSProfile(depth=10)
        self._pub = self.create_publisher(JointState, output_topic, qos)
        self.create_subscription(JointState, chassis_topic, self._on_chassis, qos)
        self.create_subscription(JointState, body_topic, self._on_body, qos)

        period_s = 1.0 / publish_rate_hz if publish_rate_hz > 0.0 else 0.01
        self.create_timer(period_s, self._publish)
        self.get_logger().info(
            f"Mux {chassis_topic} + {body_topic} -> {output_topic} @ {publish_rate_hz:.1f} Hz"
        )

    def _merge(self, msg: JointState) -> None:
        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            self._last_header = msg.header

        for idx, name in enumerate(msg.name):
            if name not in self._joint_order:
                self._joint_order.append(name)
            if idx < len(msg.position):
                self._positions[name] = float(msg.position[idx])
            if idx < len(msg.velocity):
                self._velocities[name] = float(msg.velocity[idx])
            if idx < len(msg.effort):
                self._efforts[name] = float(msg.effort[idx])

    def _on_chassis(self, msg: JointState) -> None:
        self._merge(msg)

    def _on_body(self, msg: JointState) -> None:
        self._merge(msg)

    def _publish(self) -> None:
        if not self._joint_order:
            return

        out = JointState()
        if self._last_header is not None:
            out.header = self._last_header
        else:
            out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = ""

        out.name = list(self._joint_order)
        out.position = [self._positions.get(name, 0.0) for name in out.name]
        out.velocity = [self._velocities.get(name, 0.0) for name in out.name]
        out.effort = [self._efforts.get(name, 0.0) for name in out.name]
        self._pub.publish(out)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = JointStateMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
