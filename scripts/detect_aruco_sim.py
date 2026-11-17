#!/usr/bin/env python3

"""Run Stretch's ArUco detector with MuJoCo's sensor-data subscription QoS."""

import cv2
import message_filters
import rclpy
from rclpy.qos import qos_profile_sensor_data
from stretch_core import detect_aruco_markers


def _enable_legacy_aruco():
    """Adapt OpenCV 4.6's detector API to the interface Stretch expects."""
    aruco = cv2.aruco
    if hasattr(aruco, "ArucoDetector"):
        return False

    class LegacyArucoDetector:
        def __init__(self, dictionary, parameters):
            self.dictionary = dictionary
            self.parameters = parameters

        def detectMarkers(self, image):
            return aruco.detectMarkers(
                image, self.dictionary, parameters=self.parameters
            )

        def drawDetectedMarkers(self, image, corners, ids):
            return aruco.drawDetectedMarkers(image, corners, ids)

    # The 4.6 binding exposes DetectorParameters as a type, but parameters
    # must be initialized through its factory before accessing their fields.
    aruco.DetectorParameters = aruco.DetectorParameters_create
    aruco.ArucoDetector = LegacyArucoDetector
    return True


def main():
    subscriber_class = message_filters.Subscriber

    def sensor_data_subscriber(*args, **kwargs):
        kwargs.setdefault("qos_profile", qos_profile_sensor_data)
        return subscriber_class(*args, **kwargs)

    message_filters.Subscriber = sensor_data_subscriber
    parameters_class = cv2.aruco.DetectorParameters
    legacy = _enable_legacy_aruco()
    try:
        if not legacy:
            detect_aruco_markers.main()
            return

        # Stretch's main() exits on OpenCV <4.7 before creating its node.
        # With the API adapter installed, use the same node directly.
        rclpy.init()
        node = None
        try:
            node = detect_aruco_markers.DetectArucoNode()
            node.get_logger().info(
                f"Using OpenCV {cv2.__version__} legacy ArUco compatibility adapter"
            )
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    finally:
        message_filters.Subscriber = subscriber_class
        if legacy:
            del cv2.aruco.ArucoDetector
            cv2.aruco.DetectorParameters = parameters_class


if __name__ == "__main__":
    main()
