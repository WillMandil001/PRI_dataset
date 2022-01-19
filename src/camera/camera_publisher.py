#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import copy
import cv2
import rospy
import pyrealsense2 as rs

from sensor_msgs.msg import Image, CameraInfo
from realsense_camera import RealsenseCamera

class VisionPipeline():
    
    def __init__(self, serialNum, fileMode=False, debugMode=False):
        
        self.count = 0
        self.debugMode = debugMode
        
        # Initialise camera
        if fileMode:
            pass
        else:
            self.camera = RealsenseCamera(serialNum)
        
        # Initialise planner
        self.planner = Planner()
        
        # Image segementation parameters
        self.window_size = 20      # Pixels around each mushroom to segment
        self.fixed_size = (64, 64) # Size to resize segment to. Network is
                                   # trained with (64, 64) patches

        # Setup publishing ports
        # Camera
        self.rgb_camera_info_pub    = rospy.Publisher('/camera/color/camera_info', CameraInfo, queue_size=1)
        self.img_pub                = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1)

    def start(self):
        '''
        run the vision and planning pipeline
        '''
           
        try:
            while not rospy.is_shutdown():
                # get images from camera
                colour_image, depth_image, col_fr, depth_fr = self.camera.readFrame()

                # publish images and camera info
                self.publishImages(colour_image)
                
        except:
            pass


    def publishImages(self, colour_image, depth_image):
        '''
        Publish the rgb captured by the camera along with
        the camera info to their respective ROS topics.
        
        @params:
        colour_image    - array    - a colour image
        '''
        ros_colour_img = self.bridge.cv2_to_imgmsg(colour_image, "bgr8")
        rgb_info_msg   = self.camera.get_rgb_info(ros_colour_img.header)
        
        self.stamp = \
        ros_colour_img.header.stamp = \
        rgb_info_msg.header.stamp = \

        self.img_pub.publish(ros_colour_img)
        self.rgb_camera_info_pub.publish(rgb_info_msg)                    
        
        

def main():
    rospy.init_node('VisionSystem', anonymous=False)
    serialNum    =    752112070781#rospy.get_param("%s/--serialNum"%rospy.get_name())
    debugMode    =    True #rospy.get_param("%s/--debug"%rospy.get_name())
    rc           =    VisionPipeline(serialNum, True, debugMode)
    rc.start()

if __name__ == '__main__':
   

    main()