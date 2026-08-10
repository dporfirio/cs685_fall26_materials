#!/usr/bin/env python3

"""Run Stretch object detection without Torch Hub's interactive trust prompt."""

import os
import warnings
from pathlib import Path

# YOLOv5 v7.0 still calls PyTorch's legacy autocast spelling once per
# inference. Suppress only that upstream deprecation; retain all other warnings.
warnings.filterwarnings(
    "ignore",
    message=r"`torch\.cuda\.amp\.autocast\(args\.\.\.\)` is deprecated\..*",
    category=FutureWarning,
)

# The conda OpenCV build and pip PyTorch wheel each provide an OpenMP runtime on
# macOS. Importing OpenCV first ensures its environment-level runtime is loaded
# before PyTorch, avoiding PyTorch's duplicate-runtime abort.
import cv2  # noqa: F401
import message_filters
import torch
from rclpy.qos import qos_profile_sensor_data

from stretch_deep_perception import detection_node
from stretch_deep_perception import object_detect_pytorch


class TrustedObjectDetector(object_detect_pytorch.ObjectDetector):
    def __init__(self, confidence_threshold=0.2):
        print("Loading the trusted Ultralytics YOLOv5 model...", flush=True)
        # Stretch's detector was written for the standalone YOLOv5 codebase.
        # Pin it so upstream changes do not introduce the newer `ultralytics`
        # package as an undeclared runtime dependency.
        os.environ.setdefault("TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD", "1")
        model_cache = Path(torch.hub.get_dir()) / "checkpoints"
        model_cache.mkdir(parents=True, exist_ok=True)
        previous_directory = Path.cwd()
        try:
            os.chdir(model_cache)
            self.model = torch.hub.load(
                "ultralytics/yolov5:v7.0",
                "yolov5s",
                trust_repo=True,
            )
        finally:
            os.chdir(previous_directory)
        self.confidence_threshold = confidence_threshold


def main():
    # stretch_deep_perception defaults to reliable subscriptions, but simulated
    # camera publishers use best-effort sensor-data QoS. Apply the matching QoS
    # to its three message_filters subscribers before constructing the node.
    subscriber_class = message_filters.Subscriber

    def sensor_data_subscriber(*args, **kwargs):
        kwargs.setdefault("qos_profile", qos_profile_sensor_data)
        return subscriber_class(*args, **kwargs)

    detection_node.message_filters.Subscriber = sensor_data_subscriber

    detector = TrustedObjectDetector(confidence_threshold=0.0)
    node = detection_node.DetectionNode(
        detector=detector,
        default_marker_name="object",
        node_name="DetectObjectsNode",
        topic_base_name="objects",
        fit_plane=False,
    )
    node.main()


if __name__ == "__main__":
    main()
