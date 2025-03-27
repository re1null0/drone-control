#!/usr/bin/env python3

import rclpy
from drone_controller import DroneController
import argparse


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--arm_status', type=int, required=True)
    return parser.parse_args()


def main():
    rclpy.init()
    args = parse_args()
    controller = DroneController()

    controller.wait_for_services()
    controller.wait_for_fcu()

    if args.arm_status == 1:
        controller.arm(True)
    else:
        controller.arm(False)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
