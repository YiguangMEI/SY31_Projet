#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class CameraNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('detect')

        # CvBridge is used to convert ROS messages to matrices manipulable by OpenCV
        self.bridge = CvBridge()

        # Initialize the node parameters  103 106 0 283 216 60 bgr
        self.lower_bound_blue = np.array([10,80,0])
        self.upper_bound_blue = np.array([255,216,60])

        # ranges for detetcing this color color 76,77,168 +- 20
        ##56 57 148  96 97 188 bgr
        self.lower_bound_red = np.array([20,30,100])
        self.upper_bound_red = np.array([80,90,255])
        
        #self.lower_bound_hsv = np.array([89,245,211])
        #self.upper_bound_hsv = np.array([109,255,231])
        
        # Publisher to the output topics.
        self.pub_img = rospy.Publisher('~output', Image, queue_size=10)

        # Subscriber to the input topic. self.callback is called when a message is received
        self.subscriber = rospy.Subscriber('/camera/image_raw', Image, self.callback)

    def callback(self, msg):
        '''
        Function called when an image is received.
        msg: Image message received
        img_bgr: Width*Height*3 Numpy matrix storing the image
        '''
        # Convert ROS Image -> OpenCV
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)
            return

        # TODO
        
        #img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        
        #area = cv2.contourArea(max_contour)
        # if len(contours_blue) > 0 and len(contours_red) > 0:
        #     if max_contour_blue>max_contour_red:
        #         img_contours = cv2.drawContours(img_bgr, max_contour_blue, -1, (0,255,0), 3)
        #         M = cv2.moments(max_contour_blue)
        #     elif max_contour_blue<max_contour_red:
        #         img_contours = cv2.drawContours(img_bgr, max_contour_red, -1, (0,255,0), 3) 
        #         M = cv2.moments(max_contour_red)   
        
        ###set the background
        # get size
        height, width, channels = img_bgr.shape
        img_contours = np.ones((height, width, channels), dtype=np.uint8) * 255
        #img_contours = img_bgr
        ### set filter
        img_filter_blue = cv2.inRange(img_bgr, self.lower_bound_blue, self.upper_bound_blue)
        img_filter_red = cv2.inRange(img_bgr, self.lower_bound_red, self.upper_bound_red)
        ###get contours
        contours_blue, hierarchy = cv2.findContours(img_filter_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_red, hierarchy = cv2.findContours(img_filter_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        ###get the max area
        if len(contours_blue) > 0:
            max_contour_blue=max(contours_blue, key=cv2.contourArea )
            max_blue_area=cv2.contourArea(max_contour_blue)
            ###add contours to the backgroud
            img_contours = cv2.drawContours(img_contours, max_contour_blue, -1, (0,0,255), 3)
            M_blue = cv2.moments(max_contour_blue)
            if M_blue and M_blue["m00"] != 0:
                mean_x_blue= int(M_blue["m10"]/M_blue["m00"])
                mean_y_blue= int(M_blue["m01"]/M_blue["m00"]) 
                img_contours=cv2.circle(img_contours,(mean_x_blue,mean_y_blue),20,(0,0,255),-1)      

        else :
            max_blue_area=0

        if len(contours_red) > 0:
            max_contour_red=max(contours_red, key=cv2.contourArea )
            max_red_area=cv2.contourArea(max_contour_red)
            ###add contours to the backgroud
            img_contours = cv2.drawContours(img_contours, max_contour_red, -1, (255,0,0), 3) 
            M_red = cv2.moments(max_contour_red)
            
            if M_red and M_red["m00"] != 0:
                mean_x_red= int(M_red["m10"]/M_red["m00"])
                mean_y_red= int(M_red["m01"]/M_red["m00"])
                img_contours=cv2.circle(img_contours,(mean_x_red,mean_y_red),20,(255,0,0),-1)  
            
        else :
            max_red_area=0
        
        ###add contours to the backgroud
        '''
        if max_blue_area>=max_red_area:             
            
            img_contours = cv2.drawContours(img_contours, max_contour_blue, -1, (0,0,255), 3)
            M_blue = cv2.moments(max_contour_blue)

            img_contours = cv2.drawContours(img_contours, max_contour_red, -1, (255,0,0), 3) 
            M_red = cv2.moments(max_contour_red)
        
        else:

            img_contours = cv2.drawContours(img_contours, max_contour_red, -1, (255,0,0), 3) 
            M_red = cv2.moments(max_contour_red)

            img_contours = cv2.drawContours(img_contours, max_contour_red, -1, (0,0,255), 3) 
            M_blue = cv2.moments(max_contour_red)
        '''
       
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(img_contours, "bgr8"))
        
    

if __name__ == '__main__':
    # Start the node and wait until it receives a message or stopped by Ctrl+C
    node = CameraNode()
    rospy.spin()
