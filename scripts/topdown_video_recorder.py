#!/usr/bin/env python3

"""Record a ROS image stream at a constant simulation-time playback rate."""

from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image


class TopdownVideoRecorder(Node):
    def __init__(self):
        super().__init__("topdown_video_recorder")
        self.declare_parameter("image_topic", "/overhead_camera/image_raw")
        self.declare_parameter("output_directory", "~/ament_ws/videos")
        self.declare_parameter("filename", "")
        self.declare_parameter("fps", 10.0)

        directory = Path(
            str(self.get_parameter("output_directory").value)
        ).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        filename = str(self.get_parameter("filename").value)
        if not filename:
            filename = datetime.now().strftime("exploration_%Y%m%d_%H%M%S.mp4")
        if not filename.lower().endswith(".mp4"):
            filename += ".mp4"
        self.output_path = directory / filename

        self.fps = float(self.get_parameter("fps").value)
        if self.fps <= 0.0:
            raise ValueError("fps must be positive")
        self.frame_period = 1.0 / self.fps
        self.bridge = CvBridge()
        self.writer = None
        self.output_size = None
        self.next_frame_time = None
        self.frames_written = 0
        self.last_frame = None
        self.simulation_time = None

        topic = str(self.get_parameter("image_topic").value)
        self.create_subscription(Clock, "/clock", self._clock_callback, 10)
        self.create_subscription(Image, topic, self._image_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f"Waiting for {topic}; output will be {self.output_path} at {self.fps:g} fps"
        )

    def _clock_callback(self, message):
        stamp = message.clock
        self.simulation_time = stamp.sec + stamp.nanosec * 1e-9

    def _image_callback(self, message):
        if self.simulation_time is None:
            return
        simulation_time = self.simulation_time
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")

        if self.writer is None:
            height, width = frame.shape[:2]
            if min(width, height) < 480:
                scale = 480.0 / min(width, height)
                width = int(round(width * scale))
                height = int(round(height * scale))
                frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_LINEAR)
            codec = cv2.VideoWriter_fourcc(*"mp4v")
            self.writer = cv2.VideoWriter(
                str(self.output_path), codec, self.fps, (width, height)
            )
            if not self.writer.isOpened():
                raise RuntimeError(f"Unable to open video file {self.output_path}")
            self.output_size = (width, height)
            self.next_frame_time = simulation_time
            self.get_logger().info(f"Recording {width}x{height} top-down video")

        if (frame.shape[1], frame.shape[0]) != self.output_size:
            frame = cv2.resize(
                frame, self.output_size, interpolation=cv2.INTER_LINEAR
            )

        # Use simulation timestamps rather than arrival time. When simulation is
        # slow, the resulting MP4 still plays at one simulated second per second.
        self.last_frame = frame
        while simulation_time + 1e-9 >= self.next_frame_time:
            self.writer.write(self.last_frame)
            self.frames_written += 1
            self.next_frame_time += self.frame_period

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
            duration = self.frames_written / self.fps
            self.get_logger().info(
                f"Saved {self.output_path} ({duration:.1f} simulated seconds)"
            )
        return super().destroy_node()


def main():
    rclpy.init()
    node = TopdownVideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
