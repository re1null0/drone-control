#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from mavros_msgs.msg import State, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
import numpy as np
import argparse
from nav_msgs.msg import OccupancyGrid
import time
import sys, os
from drone_controller.flightsim.simulate import simulate, Quadrotor

from drone_controller.flightsim.drone_params import quad_params




# Import from RRT-Planning-Script.py
from RRT_Planning_Script import RRT, NodeRRT
# Import from Trajectory-Smoothing.py
from Trajectory_Smoothing import smooth_trajectory
# Import from Visualization-Collision-Script.py
from Visualization_Collision_Script import VisualizationCollision

class CustomController(Node):
    def __init__(self):
        super().__init__('custom_controller')
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create rate
        self.rate = self.create_rate(20)  # 20 Hz
        
        self.current_state = State()
        self.local_position = PoseStamped()
        
        # Determines if a setpoint location is assumed to be reached if drone is within this distance
        self.arrive_offset = 1.0  # in meters
        
        # Initialize RRT planner
        self.rrt_planner = RRT()
        
        # Initialize visualization and collision detection
        self.vis_collision = VisualizationCollision()

    def state_cb(self, state):
        """
        callback function for the 'State' topic subscriber
        """
        self.current_state = state

    def local_position_pose_callback(self, data):
        self.local_position = data
        
        # Update RRT planner with current position
        if hasattr(self, 'local_position') and self.local_position:
            pos = self.local_position.pose.position
            orientation = self.local_position.pose.orientation
            _, _, yaw = self.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            self.rrt_planner.current_pose = (pos.x, pos.y, yaw)

    def map_cb(self, map_msg):
        """Callback for map updates"""
        self.rrt_planner.map_updated_ = map_msg
        self.vis_collision.map_updated_ = map_msg

    def euler_from_quaternion(self, quaternion):
        """Convert quaternion to Euler angles"""
        x, y, z, w = quaternion
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
            
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

    async def setModeOFFBoard(self):
        """
        Tries to set Flight Mode to OFFBOARD mode
        NOTE: Before entering Offboard mode, you must have already started streaming setpoints. Otherwise the mode switch will be rejected.
        """
        # exempting failsafe from lost RC to allow offboard
        param_id = "COM_RCL_EXCEPT"
        param_value = ParamValue(integer_value=1<<2, real_value=0.0)
        
        request = ParamSet.Request()
        request.param_id = param_id
        request.value = param_value
        
        future = self.set_param_srv.call_async(request)
        await future
        response = future.result()
        
        if not response.success:
            self.get_logger().error("Failure during setting param in setModeOFFBoard")

        # set the flight mode
        self.get_logger().info(f"Setting FCU mode: OFFBOARD")
        if self.current_state.mode != "OFFBOARD":
            request = SetMode.Request()
            request.base_mode = 0
            request.custom_mode = "OFFBOARD"
            
            future = self.set_mode_client.call_async(request)
            await future
            response = future.result()
            
            if not response.mode_sent:
                self.get_logger().error("Failed to send mode command")
            else:
                self.get_logger().info("Successfully set Flight Mode to OFFBOARD")

    async def setLandMode(self):
        """
        Tries to set Flight Mode to AUTO.LAND mode
        """
        self.get_logger().info(f"Setting FCU mode: AUTO.LAND")
        if self.current_state.mode != "AUTO.LAND":
            request = SetMode.Request()
            request.base_mode = 0
            request.custom_mode = "AUTO.LAND"
            
            future = self.set_mode_client.call_async(request)
            await future
            response = future.result()
            
            if not response.mode_sent:
                self.get_logger().error("Failed to send mode command")
            else:
                self.get_logger().info("Successfully set Flight Mode to AUTO.LAND")

    async def armDrone(self):
        """
        Tries to arm the drone
        """
        self.get_logger().info("Arming the drone")
        if not self.current_state.armed:
            request = CommandBool.Request()
            request.value = True
            
            future = self.arming_client.call_async(request)
            await future
            response = future.result()
            
            if not response.success:
                self.get_logger().error("Failed to send arm command")
            else:
                self.get_logger().info("Drone successfully armed")

    async def disarmDrone(self):
        """
        Tries to disarm the drone
        """
        self.get_logger().info("Disarming the drone")
        if not self.current_state.armed:
            request = CommandBool.Request()
            request.value = False
            
            future = self.arming_client.call_async(request)
            await future
            response = future.result()
            
            if not response.success:
                self.get_logger().error("Failed to send disarm command")
            else:
                self.get_logger().info("Drone successfully disarmed")

    def is_at_position(self, x, y, z, offset):
        """
        checks if drone(self.local_position) is at the given x, y, x coordinate
        NOTE: offset: meters
        """
        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    async def reach_position(self, x, y, z, vel_lin, vel_ang):
        """
        commands drone to reach the given x, y, z coordinates
        """
        # set a position setpoint
        pose = PoseStamped()
        # std_msgs/Header header
        pose.header.stamp = self.get_clock().now().to_msg()
        # geometry_msgs/Pose pose
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # create local_position_velocity message to be published
        vel = Twist()
        vel.linear = Vector3(x=vel_lin, y=vel_lin, z=vel_lin)
        vel.angular = Vector3(x=vel_ang, y=vel_ang, z=vel_ang)

        self.get_logger().info(
            f"Attempting to reach position | x: {x}, y: {y}, z: {z} | current position x: {self.local_position.pose.position.x:.2f}, "
            f"y: {self.local_position.pose.position.y:.2f}, z: {self.local_position.pose.position.z:.2f} | "
            f"linear speed: {vel_lin:.2f}, angular speed: {vel_ang:.2f}")

        while rclpy.ok():  # wait for drone to reach the desired setpoint position
            self.local_pos_pub.publish(pose)  # publish position
            self.vel_pub.publish(vel)  # publish velocity
            if self.is_at_position(pose.pose.position.x,
                                   pose.pose.position.y,
                                   pose.pose.position.z,
                                   self.arrive_offset):
                self.get_logger().info("Position reached")
                break
            await self.rate.sleep()  # rate must be higher than 2Hz or else OFFBOARD will be aborted

    async def rrt_position_control(self, positions, vel_lin, vel_ang):
        """
        Uses RRT for path planning to navigate between waypoints
        """
        prev_state = self.current_state

        # NOTE: Before entering Offboard mode, you must have already started streaming setpoints. Otherwise the mode switch will be rejected.
        # send a few setpoints before starting
        for i in range(100):
            self.local_pos_pub.publish(self.local_position)
            await self.rate.sleep()

        await self.setModeOFFBoard()  # set Flight mode to OFFBOARD
        await self.armDrone()  # arm the drone before takeoff

        while rclpy.ok():
            # log changes in the arm status & flight mode status
            if prev_state.armed != self.current_state.armed:
                self.get_logger().info(f"Vehicle armed: {self.current_state.armed}")
            if prev_state.mode != self.current_state.mode:
                self.get_logger().info(f"Current mode: {self.current_state.mode}")
            prev_state = self.current_state

            # check if the drone is armed and in OFFBOARD flight mode before flying to setpoints
            if self.current_state.armed and self.current_state.mode == "OFFBOARD":
                # set out to reach waypoints one by one in order
                for i in range(len(positions)):
                    goal_x, goal_y, goal_z = positions[i]
                    start_x, start_y = self.local_position.pose.position.x, self.local_position.pose.position.y
                    
                    # Plan path using RRT
                    self.get_logger().info(f"Planning path to waypoint {i+1}: ({goal_x}, {goal_y}, {goal_z})")
                    path, tree = self.rrt_planner.plan_rrt(start_x, start_y, goal_x, goal_y)
                    
                    # Visualize RRT tree and path
                    self.vis_collision.visualize_rrt(path, tree)
                    
                    if path:
                        # Smooth the path
                        smoothed_path = smooth_trajectory(path)
                        self.get_logger().info(f"Path found with {len(smoothed_path)} points")
                        
                        # Follow the smoothed path
                        for point in smoothed_path:
                            await self.reach_position(point[0], point[1], goal_z, vel_lin, vel_ang)
                    else:
                        self.get_logger().warn(f"No path found to waypoint {i+1}, moving directly")
                        await self.reach_position(goal_x, goal_y, goal_z, vel_lin, vel_ang)

                # land the drone
                await self.setLandMode()
                # disarm the drone
                await self.disarmDrone()
                break

            await self.rate.sleep()


# custom PID controller

    # async def run_controller(self):
    #     quadrotor = Quadrotor(quad_params)

    #     initial_position = self.get_initial_pose_state()
    #     my_se3_control = se3_control.SE3Control(quad_params) # GIVEN
    #     points = np.array([
    #         [0, 0, 0],
    #         [1.5, 0, 0],
    #         [1.5, 1.5, 0],
    #         [1.5, 1.5, 1.5],
    #         [-1.5, 1.5, 1.5],
    #         [-1.5, 1.5, -1.5],
    #         [-1.5, 1.5, -1.5],
    #         [-1.5, 0, -1.5],
    #         [0, 0, -1.5],
    #         [0, 0, 0]
    #     ])
    #     my_traj = waypoint_traj.WaypointTraj(points)
    #     t_final = 60


    #     (time, state, control, flat, exit_status) = simulate(initial_position, quadrotor, my_se3_control, my_traj, t_final)



    # def get_initial_pose_state(self):
    #     """
    #         Returns a dictionary with the drone's initial:
    #         - Position (x, y, z)
    #         - Velocity (vx, vy, vz)
    #         - Orientation quaternion (qx, qy, qz, qw)
    #         - Angular velocity (wx, wy, wz)

    #         Assumes self.local_position has been populated and the appropriate IMU/velocity data is stored.`
    #     """
    #     try:
    #         pos = self.local_position.pose.position
    #         orient = self.local_position.pose.orientation

    #         # You may need to store these yourself via subscribers (if not already available)
    #         # e.g., self.latest_velocity from /mavros/local_position/velocity_local
    #         #       self.latest_imu from /mavros/imu/data

    #         vx, vy, vz = self.latest_velocity.twist.linear.x, self.latest_velocity.twist.linear.y, self.latest_velocity.twist.linear.z
    #         wx, wy, wz = self.latest_imu.angular_velocity.x, self.latest_imu.angular_velocity.y, self.latest_imu.angular_velocity.z

    #         return {
    #             "position": (pos.x, pos.y, pos.z),
    #             "velocity": (vx, vy, vz),
    #             "quaternion": (orient.x, orient.y, orient.z, orient.w),
    #             "angular_velocity": (wx, wy, wz)
    #         }

    #     except AttributeError:
    #         self.get_logger().warn("Initial pose state data not yet fully available.")
    #         return None


    async def position_control(self, positions, vel_lin, vel_ang):
        """
        Original position control method (without RRT)
        """
        prev_state = self.current_state

        # NOTE: Before entering Offboard mode, you must have already started streaming setpoints. Otherwise the mode switch will be rejected.
        # send a few setpoints before starting
        for i in range(100):
            self.local_pos_pub.publish(self.local_position)
            await self.rate.sleep()

        await self.setModeOFFBoard()  # set Flight mode to OFFBOARD
        await self.armDrone()  # arm the drone before takeoff

        while rclpy.ok():
            # log changes in the arm status & flight mode status
            if prev_state.armed != self.current_state.armed:
                self.get_logger().info(f"Vehicle armed: {self.current_state.armed}")
            if prev_state.mode != self.current_state.mode:
                self.get_logger().info(f"Current mode: {self.current_state.mode}")
            prev_state = self.current_state

            # check if the drone is armed and in OFFBOARD flight mode before flying to setpoints
            if self.current_state.armed and self.current_state.mode == "OFFBOARD":
                # set out to reach waypoints one by one in order
                for i in range(len(positions)):
                    await self.reach_position(positions[i][0], positions[i][1], positions[i][2], vel_lin, vel_ang)

                # land the drone
                await self.setLandMode()
                # disarm the drone
                await self.disarmDrone()
                break

            await self.rate.sleep()


    async def execute_mission(self, positions, vel_lin, vel_ang, use_rrt=False):
        """
        Executes a full mission by flying through all waypoints in order.
        """
        self.get_logger().info("Initializing mission...")
        self.checkForTopics()

        services_ready = await self.checkForServices()
        if not services_ready:
            self.get_logger().error("Service initialization failed.")
            return

        fcu_ready = await self.checkForFCU()
        if not fcu_ready:
            self.get_logger().error("FCU connection failed.")
            return

        self.get_logger().info("Starting mission...")

        if use_rrt:
            await self.rrt_position_control(positions, vel_lin, vel_ang)
        else:
            await self.position_control(positions, vel_lin, vel_ang)

        self.get_logger().info("Mission completed.")



    async def checkForServices(self):
        """
        Tries to initialize the services
        """
        service_timeout = 30
        self.get_logger().info("Waiting for ROS services")
        
        try:
            # Create service clients
            self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
            self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
            self.set_param_srv = self.create_client(ParamSet, 'mavros/param/set')
            
            # Wait for services to be available
            start_time = time.time()
            while not self.arming_client.wait_for_service(timeout_sec=1.0):
                if time.time() - start_time > service_timeout:
                    raise RuntimeError("Service not available")
                self.get_logger().info("Waiting for arming service...")
                
            start_time = time.time()
            while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
                if time.time() - start_time > service_timeout:
                    raise RuntimeError("Service not available")
                self.get_logger().info("Waiting for set_mode service...")
                
            start_time = time.time()
            while not self.set_param_srv.wait_for_service(timeout_sec=1.0):
                if time.time() - start_time > service_timeout:
                    raise RuntimeError("Service not available")
                self.get_logger().info("Waiting for param service...")
                
            self.get_logger().info("ROS services are up")
            self.get_logger().info("Successfully connected to the ROS services.")
            
        except Exception as e:
            self.get_logger().error(f"Failed to connect to services: {e}")
            return False
            
        return True

    def checkForTopics(self):
        """
        Tries to initialize the Subscribers/Publishers
        """
        try:
            # Create publishers
            self.local_pos_pub = self.create_publisher(
                PoseStamped, 'mavros/setpoint_position/local', 10)
            self.vel_pub = self.create_publisher(
                Twist, 'mavros/setpoint_velocity/cmd_vel_unstamped', 10)
                
            # Create subscribers
            self.state_sub = self.create_subscription(
                State, 'mavros/state', self.state_cb, 10)
            self.local_pos_sub = self.create_subscription(
                PoseStamped, 'mavros/local_position/pose', self.local_position_pose_callback, 10)
            self.map_sub = self.create_subscription(
                OccupancyGrid, '/map', self.map_cb, 10)
                
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Publishers/Subscribers: {e}")
            return False

    async def checkForFCU(self):
        """
        Tries to establish a FCU connection
        """
        # wait for FCU connection
        self.get_logger().info("Waiting for FCU connection")
        while not self.current_state.connected:
            await self.rate.sleep()
        self.get_logger().info("FCU connection established")
        return True

def parsePositions():
    """
    Parses position arguments passed in as command line arguments with --positions flag.
    e.g. --positions x1 y1 z1 x2 y2 z2 ... xn yn zn ; where x,y,z are floats

    Sample call:
    $python3 my_mission_script.py --positions 10 10 10 50 50 20 50 -50 20 -50 -50 20 0 0 20

    Sample return:
    return [(10.0, 10.0, 10.0), (50.0, 50.0, 20.0), (50.0, -50.0, 20.0), (-50.0, -50.0, 20.0), (0.0, 0.0, 20.0)]
    """
    CLI=argparse.ArgumentParser()
    CLI.add_argument(
        "--positions", # name on the CLI - drop the `--` for positional/required parameters
        nargs="*", # 0 or more values expected => creates a list
        type=float,
        default=[], # default if nothing is provided,
        required=True,
    )

    CLI.add_argument(
        "--linear_velocity", # name on the CLI - drop the `--` for positional/required parameters
        type=float,
        required=True
    )

    CLI.add_argument(
        "--angular_velocity", # name on the CLI - drop the `--` for positional/required parameters
        type=float,
        required=True,
    )

    CLI.add_argument(
        "--use_rrt", # flag to use RRT planning
        action="store_true",
        default=False,
        help="Use RRT for path planning"
    )

    args = CLI.parse_args()

    positions = args.positions
    if len(positions) > 0 and len(positions)%3 == 0:
        pos = []
        for i in range(0,len(positions)-2, 3):
            x = positions[i]
            y = positions

    return args

def main(args=None):
    rclpy.init(args=args)
    args = parsePositions()
    pos_floats = args.positions
    pos_tuples = list(zip(pos_floats[::3], pos_floats[1::3], pos_floats[2::3]))
    vel_lin = args.linear_velocity
    vel_ang = args.angular_velocity
    use_rrt = args.use_rrt


    controller = CustomController()
    try:
        rclpy.spin_until_future_complete(controller, controller.execute_mission(pos_tuples, vel_lin, vel_ang, use_rrt))
    except KeyboardInterrupt:
        controller.get_logger().info("Mission aborted by user.")
    finally:
        controller.destroy_node()
        rclpy.shutdown()
