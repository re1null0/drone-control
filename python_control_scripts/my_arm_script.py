#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
import argparse


class CustomController(Node):
    def __init__(self):
        super().__init__('offb_node')

        self.freq = 20.0  # in Hz
        self.current_state = State()
        self.local_position = PoseStamped()
        self.initial_pos_acquired = False
        self.initial_position = PoseStamped()
        self.arrive_offset = 1.0  # in meters

        # Set up publishers and subscribers
        self.publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.local_pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_position_pose_callback, 10)

        self.timer = self.create_timer(1.0 / self.freq, self.timer_callback)

        # Initialize service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.set_param_srv = self.create_client(ParamSet, '/mavros/param/set')

    def timer_callback(self):
        pass  # Placeholder for periodic tasks if needed

    def state_cb(self, state):
        self.current_state = state

    def local_position_pose_callback(self, data):
        self.local_position = data
        if not self.initial_pos_acquired:
            self.initial_position = data
            self.initial_pos_acquired = True

    def wait_for_services(self):
        self.get_logger().info("Waiting for ROS2 services...")
        for client, name in [
            (self.arming_client, "arming"),
            (self.set_mode_client, "set_mode"),
            (self.set_param_srv, "param_set")
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for {name} service...")

        self.get_logger().info("All services available.")

    def armDrone(self, arm_status):
        arm_status = bool(arm_status)
        self.get_logger().info(f"Setting arming status to: {arm_status}")
        if self.current_state.armed != arm_status:
            request = CommandBool.Request()
            request.value = arm_status
            future = self.arming_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info("Drone armed successfully")
                else:
                    self.get_logger().error("Failed to arm drone")
            else:
                self.get_logger().error("Service call to arm failed")

    def checkForFCU(self):
        self.get_logger().info("Waiting for FCU connection...")
        while not self.current_state.connected:
            rclpy.spin_once(self)
        self.get_logger().info("FCU connection established.")

    def position_control(self, arm_status):
        self.armDrone(arm_status)


def parsePositions():
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm_status", type=int, required=True)
    args = parser.parse_args()
    return args.arm_status


def main(args=None):
    rclpy.init(args=args)
    arm_status = parsePositions()

    try:
        controller = CustomController()
        controller.wait_for_services()
        controller.checkForFCU()
        controller.position_control(arm_status)
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down node...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
