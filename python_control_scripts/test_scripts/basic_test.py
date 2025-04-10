#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, ParamSet, CommandTOL
from geometry_msgs.msg import PoseStamped
import asyncio

class GuidedTakeoffNode(Node):
    def __init__(self):
        super().__init__('guided_takeoff_node')
        self.state = State()

        # Subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)

        # Publishers
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.param_client = self.create_client(ParamSet, '/mavros/param/set')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        # Pose for dummy setpoints
        self.pose = PoseStamped()
        self.pose.pose.position.z = 1.5

    def state_cb(self, msg):
        self.state = msg

async def main():
    rclpy.init()
    node = GuidedTakeoffNode()

    try:
        # 🔌 Wait for FCU
        node.get_logger().info("📡 Waiting for FCU connection...")
        while not node.state.connected:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info("✅ Connected to FCU")

        # 🔧 Wait for services
        for srv in [node.arming_client, node.set_mode_client, node.takeoff_client]:
            while not srv.wait_for_service(timeout_sec=1.0):
                node.get_logger().info(f"⏳ Waiting for service: {srv.srv_name}")

        # # 🛡️ Disable arming checks for indoor test
        # node.get_logger().info("🛠️ Disabling arming checks...")
        # param_req = ParamSet.Request()
        # param_req.param_id = "ARMING_CHECK"
        # param_req.value.integer_value = 0
        # param_req.value.real_value = 0.0
        # await node.param_client.call_async(param_req)

        # 📤 Send dummy setpoints before mode change
        node.get_logger().info("📤 Sending dummy setpoints...")
        for _ in range(100):
            node.local_pos_pub.publish(node.pose)
            rclpy.spin_once(node, timeout_sec=0.01)
            await asyncio.sleep(0.01)

        # 🎛️ Set GUIDED mode
        node.get_logger().info("🎛️ Setting mode to GUIDED...")
        mode_req = SetMode.Request()
        mode_req.custom_mode = "AUTO"
        await node.set_mode_client.call_async(mode_req)

        while node.state.mode != "AUTO":
            await asyncio.sleep(0.2)
            rclpy.spin_once(node)

        node.get_logger().info("Mode set to GUIDED")

        # 🔓 Arm the drone
        arm_req = CommandBool.Request()
        arm_req.value = True
        node.get_logger().info("🔓 Sending arm command...")
        await node.arming_client.call_async(arm_req)

        while not node.state.armed:
            await asyncio.sleep(0.2)
            rclpy.spin_once(node)

        node.get_logger().info("✅ Drone armed")

        # 🚁 Takeoff
        node.get_logger().info("🚁 Sending takeoff command...")
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = 1.5
        await node.takeoff_client.call_async(takeoff_req)

        await asyncio.sleep(6)  # Hover for a few seconds

        # 🛬 Land
        node.get_logger().info("🛬 Sending land command...")
        land_req = SetMode.Request()
        land_req.custom_mode = "LAND"
        await node.set_mode_client.call_async(land_req)

        while node.state.mode != "LAND":
            await asyncio.sleep(0.2)
            rclpy.spin_once(node)

        node.get_logger().info("✅ Landing initiated")

        # Wait until landed + disarm
        await asyncio.sleep(5)
        disarm_req = CommandBool.Request()
        disarm_req.value = False
        node.get_logger().info("🔒 Disarming...")
        await node.arming_client.call_async(disarm_req)

        node.get_logger().info("✅ Flight complete!")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    asyncio.run(main())
