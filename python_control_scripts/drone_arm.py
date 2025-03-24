#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
import argparse


class ArmDisarmNode(Node):
    def __init__(self, arm_status):
        super().__init__('arm_disarm_node')

        self.arm_status = bool(arm_status)
        self.current_state = State()

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10
        )

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.get_logger().info("Waiting for /mavros/cmd/arming service...")
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Still waiting for arming service...")

        self.get_logger().info("Waiting for FCU connection...")
        while not self.current_state.connected:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Connected to FCU.")

        self.arm_drone()

    def state_cb(self, msg):
        self.current_state = msg

    def arm_drone(self):
        request = CommandBool.Request()
        request.value = self.arm_status
        action = "Arming" if self.arm_status else "Disarming"
        self.get_logger().info(f"{action} drone...")

        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f"{action} succeeded.")
            else:
                self.get_logger().error(f"{action} failed.")
        else:
            self.get_logger().error("No response from arming service.")


def parse_args():
    parser = argparse.ArgumentParser(description='Arm or disarm a drone via MAVROS')
    parser.add_argument('--arm_status', type=int, required=True, help='1 to arm, 0 to disarm')
    return parser.parse_args()


def main(args=None):
    rclpy.init(args=args)
    cli_args = parse_args()

    try:
        node = ArmDisarmNode(cli_args.arm_status)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
