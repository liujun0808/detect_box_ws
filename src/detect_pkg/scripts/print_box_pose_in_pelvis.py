#!/usr/bin/env python3
"""Print the simulated box pose expressed in the robot pelvis frame."""

from __future__ import annotations

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class BoxPoseInPelvisPrinter(Node):
    """Looks up T_pelvis_box from TF and prints it at a fixed rate."""

    def __init__(self) -> None:
        super().__init__("box_pose_in_pelvis_printer")
        self.declare_parameter("pelvis_frame", "pelvis")
        self.declare_parameter("box_frame", "table_object_box")
        self.declare_parameter("print_rate_hz", 10.0)

        self._pelvis_frame = self.get_parameter("pelvis_frame").value
        self._box_frame = self.get_parameter("box_frame").value
        print_rate_hz = float(self.get_parameter("print_rate_hz").value)
        if print_rate_hz <= 0.0:
            raise ValueError("print_rate_hz must be greater than zero")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_error = ""
        self._timer = self.create_timer(1.0 / print_rate_hz, self._print_pose)
        self.get_logger().info(
            f"Waiting for TF {self._pelvis_frame} <- {self._box_frame}; "
            f"printing at {print_rate_hz:.1f} Hz"
        )

    def _print_pose(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._pelvis_frame,
                self._box_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as error:
            error_text = str(error)
            if error_text != self._last_error:
                self.get_logger().warn(
                    f"Cannot transform {self._box_frame} into {self._pelvis_frame}: {error_text}"
                )
                self._last_error = error_text
            return

        self._last_error = ""
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.get_logger().info(
            "T_pelvis_box: "
            f"position[m]=[{translation.x:.4f}, {translation.y:.4f}, {translation.z:.4f}], "
            "orientation[xyzw]="
            f"[{rotation.x:.6f}, {rotation.y:.6f}, {rotation.z:.6f}, {rotation.w:.6f}]"
        )


def main() -> None:
    rclpy.init()
    node = BoxPoseInPelvisPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
