#!/usr/bin/env python3

import rclpy
from my_mission_script import CustomController
from geometry_msgs.msg import Twist

import argparse
import asyncio

async def run_velocity_test(controller, direction: str, velocity: float, duration: float, altitude: float):
    await controller.checkForServices()
    controller.checkForTopics()
    await controller.checkForFCU()

    # dummy setpoints 
    for _ in range(100):
        controller.local_pos_pub.publish(controller.local_position)
        await controller.rate.sleep()

    await controller.setModeOFFBoard()
    await controller.armDrone()

    # taking off
    controller.get_logger().info(f"Taking off to {altitude}m...")
    await controller.reach_position(
        controller.local_position.pose.position.x,
        controller.local_position.pose.position.y,
        altitude,
        vel_lin=0.3,
        vel_ang=0.0
    )

    # velocity command
    vel_msg = Twist()
    if direction == "x":
        vel_msg.linear.x = velocity
    elif direction == "y":
        vel_msg.linear.y = velocity
    elif direction == "z":
        vel_msg.linear.z = velocity
    else:
        controller.get_logger().error("Invalid direction. Use x, y, or z.")
        return

    ticks = int(duration * 20)  # 20Hz rate
    controller.get_logger().info(f"Flying {velocity} m/s along {direction}-axis for {duration} seconds")
    for _ in range(ticks):
        controller.vel_pub.publish(vel_msg)
        await controller.rate.sleep()

    # Stop
    controller.get_logger().info("Stopping drone motion")
    controller.vel_pub.publish(Twist())

    # Land
    await controller.setLandMode()
    await controller.disarmDrone()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--direction", type=str, choices=["x", "y", "z"], required=True)
    parser.add_argument("--velocity", type=float, required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--altitude", type=float, default=1.5)
    args = parser.parse_args()

    rclpy.init()
    controller = CustomController()

    try:
        asyncio.run(run_velocity_test(
            controller,
            args.direction,
            args.velocity,
            args.duration,
            args.altitude
        ))
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
