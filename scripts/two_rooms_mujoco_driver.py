#!/usr/bin/env python3

"""ROS Stretch driver for two empty rooms connected by a short hallway."""

from minimal_mujoco_driver import MinimalStretchMujocoSimulator
import stretch_mujoco_driver.stretch_mujoco_driver as driver


class TwoRoomsStretchMujocoSimulator(MinimalStretchMujocoSimulator):
    scene_name = "two_rooms.xml"


def main():
    driver.StretchMujocoSimulator = TwoRoomsStretchMujocoSimulator
    driver.main()


if __name__ == "__main__":
    main()
