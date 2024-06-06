#!/usr/bin/env python3
 
import numpy as np
import rospy
 
# Type of input and output messages
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
 
PC2FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('c', 12, PointField.INT16, 1)
]
 
class Transformer:
    def __init__(self):
        rospy.init_node('transformer')
        self.pub_pc2 = rospy.Publisher('/lidar/points', PointCloud2, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.callback)
        rospy.Subscriber('/pose_gyro', PoseStamped, self.callback_odom)
        self.X = 0
        self.Y = 0
        self.theta = 0
        self.coords = []
    
    def callback_odom(self, msg):
        self.X = msg.pose.position.x 
        self.Y = msg.pose.position.y 
        self.theta = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]
    
    def callback(self, msg):
 
        for i, theta in enumerate(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)):
            # ToDo: Remove points too close
            if msg.ranges[i] >= 0.1:
                x=msg.ranges[i]*np.cos(theta)
                y=msg.ranges[i]*np.sin(theta)
                
                tmp = np.array([[x],[y],[1]])
                
                transfo_matrix = np.array([[np.cos(self.theta),-np.sin(self.theta),self.X], [np.sin(self.theta),np.cos(self.theta),self.Y],[0,0,1]])
                
                coords_O=np.dot(transfo_matrix,tmp)
                
                self.coords.append([coords_O[0][0],coords_O[1][0]])
            
        pc2 = create_cloud(msg.header, PC2FIELDS, [[x,y,0,0] for x,y in self.coords])
        self.pub_pc2.publish(pc2)
        
    def run(self):
        rospy.spin()
 
if __name__ == '__main__':
    node = Transformer()
    node.run() 
