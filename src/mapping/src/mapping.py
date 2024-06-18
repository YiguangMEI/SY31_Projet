#!/usr/bin/env python3
 
import numpy as np
import rospy
 
# Type of input and output messages
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.point_cloud2 import read_points
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
        rospy.init_node('mapping')
        self.pub_pc2 = rospy.Publisher('/lidar/map', PointCloud2, queue_size=10)

        #rospy.Subscriber('/scan', LaserScan, self.callback)
        #on va recupere les pojnts clustrisé d'ici: pub_clusters = rospy.Publisher('/lidar/clusters', PointCloud2, queue_size=10)
        rospy.Subscriber('lidar/clusters', PointCloud2, self.callback)
        rospy.Subscriber('/pose_gyro', PoseStamped, self.callback_odom1)
        rospy.Subscriber('/pose_enco', PoseStamped, self.callback_odom2)
        self.X_robot  = 0
        self.Y_robot  = 0
        self.theta_robot = 0
        self.coords = []
        self.odom_time = None
        self.points_time = None
s
    
    #how to comment multiline in python: ctrl+/

    def callback_odom2(self, msg):
        self.X_robot = msg.pose.position.x 
        self.Y_robot = msg.pose.position.y 
        
        self.odom_time = msg.header.stamp

    def callback_odom1(self, msg):
        self.theta_robot = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]
        self.odom_time = msg.header.stamp
    
    # def callback(self, msg):
 
    #     for i, theta in enumerate(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)):
    #         # ToDo: Remove points too close
    #         if 3>=msg.ranges[i] >= 0.1:
    #             x=msg.ranges[i]*np.cos(theta)
    #             y=msg.ranges[i]*np.sin(theta)
                
    #             tmp = np.array([[x],[y],[1]])
                
    #             transfo_matrix = np.array([[np.cos(self.theta_robot),-np.sin(self.theta_robot),self.X_robot], [np.sin(self.theta_robot),np.cos(self.theta_robot),self.Y_robot],[0,0,1]])
                
    #             coords_O=np.dot(transfo_matrix,tmp)
                
    #             self.coords.append([coords_O[0][0],coords_O[1][0]])
            
    #     pc2 = create_cloud(msg.header, PC2FIELDS, [[x,y,0,0] for x,y in self.coords])
    #     self.pub_pc2.publish(pc2)

    #callback qui utilise des x y qui viennent de clusterer, donc des PointCloud2
    
    def callback(self, msg):
        self.points_time = msg.header.stamp

        # Check if the timestamps are close enough (synchronization)
        if self.odom_time is not None and abs(self.odom_time.to_sec() - self.points_time.to_sec()) < 0.05:
            for x, y, _, _ in read_points(msg):
                tmp = np.array([[x], [y], [1]])
                transfo_matrix = np.array([
                    [np.cos(self.theta_robot), -np.sin(self.theta_robot), self.X_robot],
                    [np.sin(self.theta_robot), np.cos(self.theta_robot), self.Y_robot],
                    [0, 0, 1]
                ])
                coords_O = np.dot(transfo_matrix, tmp)
                #filter coords
                self.coords.append([coords_O[0][0], coords_O[1][0]])
            pc2 = create_cloud(msg.header, PC2FIELDS, [[x, y, 0, 0] for x, y in self.coords])
            self.pub_pc2.publish(pc2)
        else:
            rospy.logwarn("Timestamps are not synchronized")


        
    def run(self):
        rospy.spin()
 
if __name__ == '__main__':
    node = Transformer()
    node.run() 
