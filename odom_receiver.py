#!/usr/bin/env python3
import rospy
import socket
import pickle
from nav_msgs.msg import Odometry

# Choose the same PORT you used on the intermediate
# For UDP, we just need to bind to the local address *or* 0.0.0.0 if you prefer
LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5000

def main():
    rospy.init_node("odom_receiver_node")
    pub = rospy.Publisher("/vicon/drone_odom", Odometry, queue_size=10)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))

    rospy.loginfo("Odom receiver node started on port %d" % LISTEN_PORT)

    while not rospy.is_shutdown():
        # For UDP, receive up to ~65k bytes
        data, addr = sock.recvfrom(65535)
        odom_msg = pickle.loads(data)
        pub.publish(odom_msg)

if __name__ == "__main__":
    main()
