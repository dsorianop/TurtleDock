#!/usr/bin/env python3

#headers in progress
#yaw converted to degrees from radians for easy interpretation
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class yaw:
    def __init__(self):
        #starts a ROS node named yaw_tracker
        rospy.init_node('yaw_tracker', anonymous=True)

        #there is no current yaw in degrees
        self.yaw_deg = None

        #/odom topic contains robot pose and orientation data
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.loginfo("Node has been initialized. Listening and awaiting odometry updates.")
        rospy.spin()

    def odom_callback(self, msg):
        #retrieves quaternion components from message
        orientation_data = msg.pose.pose.orientation
        
        #Euler's roll, pitch, and yaw angles
        quaternion = [
            orientation_data.x,
            orientation_data.y,
            orientation_data.z,
            orientation_data.w
        ]

        #quaternions are harder for humans to interpret, so they are converted to Euler angles
        roll, pitch, yaw_rad = euler_from_quaternion(quaternion)

        #data is converted from radians to degrees
        yaw_deg = math.degrees(yaw_rad)

        #revise this
        yaw_deg = (yaw_deg + 360) % 360

        #refreshes variable
        self.yaw_deg = yaw_deg

        #later, the value is to be stored elsewhere

        #log most recent yaw
        rospy.loginfo(f"Yaw (deg): {self.yaw_deg:.2f}")

#haven't gotten the chance to test the exceptions
if __name__ == '__main__':
    try:
        yaw()
    except rospy.ROSInterruptException:
        #testing alternative to logging
        pass
