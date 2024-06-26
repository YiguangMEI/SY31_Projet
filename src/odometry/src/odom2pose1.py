#!/usr/bin/env python3

import numpy as np
import math 
import rospy
from geometry_msgs.msg import PoseStamped
from turtlebot3_msgs.msg import SensorState
from sensor_msgs.msg import Imu, MagneticField

from tf.transformations import quaternion_from_euler

def coordinates_to_message(x, y, O, t):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    [msg.pose.orientation.x,
    msg.pose.orientation.y,
    msg.pose.orientation.z,
    msg.pose.orientation.w] = quaternion_from_euler(0.0, 0.0, O)
    msg.header.stamp = t
    msg.header.frame_id = 'base_scan'
    return msg

class Odom2PoseNode:
    def __init__(self):
        rospy.init_node('odom2pose')

        # Constants
        self.ENCODER_RESOLUTION = 4096
        self.WHEEL_RADIUS = 0.033
        self.WHEEL_SEPARATION = 0.160
        self.MAG_OFFSET = np.pi/2.0-0.07

        # Variables
        self.x_odom, self.y_odom, self.O_odom = 0, 0, 0
        self.x_gyro, self.y_gyro, self.O_gyro = 0, 0, 0
        self.x_magn, self.y_magn, self.O_magn = 0, 0, 0
        self.prev_left_encoder = 0
        self.prev_right_encoder = 0
        self.prev_gyro_t = 0
        self.v = 0

        # Publishers
        self.pub_enco = rospy.Publisher('/pose_enco', PoseStamped, queue_size=0)
        self.pub_gyro = rospy.Publisher('/pose_gyro', PoseStamped, queue_size=0)
        self.pub_magn = rospy.Publisher('/pose_magn', PoseStamped, queue_size=0)

        # Subscribers
        self.sub_gyro = rospy.Subscriber('/imu', Imu, self.callback_gyro)
        self.sub_enco = rospy.Subscriber('/sensor_state', SensorState, self.callback_enco)
        self.sub_magn = rospy.Subscriber('/magnetic_field', MagneticField, self.callback_magn)

    def callback_enco(self, sensor_state):
        # Compute the differential in encoder count
        d_left_encoder = sensor_state.left_encoder-self.prev_left_encoder
        d_right_encoder = sensor_state.right_encoder-self.prev_right_encoder
        if self.prev_left_encoder == 0:
            self.prev_left_encoder = sensor_state.left_encoder
            self.prev_right_encoder = sensor_state.right_encoder
            return
        self.prev_right_encoder = sensor_state.right_encoder
        self.prev_left_encoder = sensor_state.left_encoder

        # TODO: Compute the linear and angular velocity (self.v and w)
        
        r_speed = (2* math.pi/self.ENCODER_RESOLUTION * d_right_encoder)/ (1/20) * self.WHEEL_RADIUS
        l_speed = (2*math.pi/self.ENCODER_RESOLUTION * d_left_encoder)/ (1/20) * self.WHEEL_RADIUS
        
        self.v = (r_speed + l_speed)/2
        self.w = (r_speed - l_speed)/self.WHEEL_SEPARATION 

        # TODO: Update x_odom, y_odom and O_odom accordingly
        
        self.x_odom = self.x_odom + self.v * 1/20 * math.cos(self.O_odom)
        self.y_odom = self.y_odom + self.v * 1/20 * math.sin(self.O_odom)
        self.O_odom = self.O_odom + self.w  *1/20

        msg = coordinates_to_message(self.x_odom, self.y_odom, self.O_odom, sensor_state.header.stamp)
        self.pub_enco.publish(msg)

    def callback_gyro(self, gyro):
        if self.v == 0:
            return

        # Compute the elapsed time
        t = gyro.header.stamp.to_sec()
        dt = t - self.prev_gyro_t
        
        if dt <= 0:
            return  # Handle negative or zero time difference

        # Update previous time stamp
        self.prev_gyro_t = t

        # TODO: compute the angular velocity
        
        angular_velocity_z = gyro.angular_velocity.z
        self.w = angular_velocity_z

        # TODO: update O_gyro, x_gyro and y_gyro accordingly (using self.v)
        
        self.O_gyro += self.w * dt
        self.x_gyro += self.v * dt * math.cos(self.O_gyro)
        self.y_gyro += self.v * dt * math.sin(self.O_gyro)

        

        msg = coordinates_to_message(self.x_odom, self.y_odom,self.O_gyro, gyro.header.stamp)
        self.pub_gyro.publish(msg)

    def callback_magn(self, magnetic_field):
        if self.v == 0:
            return

        # TODO: compute the angle O_magn from magnetic fields (using MAG_OFFSET)
        
        self.O_magn = (np.arctan2(magnetic_field.magnetic_field.y,magnetic_field.magnetic_field.x) + self.MAG_OFFSET ) 

        # TODO: update O_magn, x_magn and y_magn accordingly (using self.v)

        self.x_magn = self.x_magn  + self.v  * math.cos(self.O_magn) * 1/20
        self.y_magn = self.y_magn+ self.v * math.sin(self.O_magn)* 1/20
        

        msg = coordinates_to_message(self.x_magn, self.y_magn, self.O_magn, magnetic_field.header.stamp)
        self.pub_magn.publish(msg)

if __name__ == '__main__':
    node = Odom2PoseNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
