#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import rospy
import time
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, RegionOfInterest, PointCloud2, PointField
#import open3d
#from open3d.open3d.geometry import Image as rsImage
#from open3d.open3d.camera import PinholeCameraIntrinsic, PinholeCameraIntrinsicParameters

#from std_msgs.msg import Int16MultiArray
import argparse
import sys




class RealsenseCamera(object):
    def __init__(self, serialNum):
       
        self.bridge = CvBridge()
        # self.frame_rate = 30
        # self.height = 720
        # self.width = 1280

        self.color_frame_rate = 60
        self.color_height = 480
        self.color_width = 640

        self.depth_frame_rate = 60
        self.depth_height = 270
        self.depth_width = 480
        
        self.DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07"]
        
        self.pipeline = rs.pipeline()
        self.profile = self.pipeline.start(self.load_config(serialNum))
        self.align = rs.align(rs.stream.color)
        self.enable_advanced_mode()

        super(RealsenseCamera, self).__init__()


    def load_config(self, serialNum):
        
        # Create a config and configure the pipeline to stream
        config = rs.config()
        config.enable_device("%s"%str(serialNum).strip("'"))
        #profile = config.resolve(self.pipeline) # does not start streaming
        config.enable_stream(rs.stream.depth, self.depth_width, self.depth_height, rs.format.z16, self.depth_frame_rate)
        config.enable_stream(rs.stream.color, self.color_width, self.color_height, rs.format.bgr8, self.color_frame_rate)
        return config

    def get_camera_info_msg(self, msg_header, k, p):
        cam_info = CameraInfo()
        cam_info.header = msg_header
        cam_info.header.frame_id = "/camera_color_optical_frame"
        cam_info.height = self.color_height
        cam_info.width = self.color_width
        cam_info.distortion_model = "plumb_bob"
        cam_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        cam_info.K = k
        cam_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        cam_info.P = p
        cam_info.binning_x = 0
        cam_info.binning_y = 0
        cam_info.roi = RegionOfInterest()
        cam_info.roi.x_offset = 0
        cam_info.roi.y_offset = 0
        cam_info.roi.height = 0
        cam_info.roi.width = 0
        cam_info.roi.do_rectify = False
        return cam_info

    def get_rgb_info(self, msg_header):
        k = [617.4435424804688, 0.0, 317.70989990234375, 0.0, 617.878662109375, 238.01991271972656, 0.0, 0.0, 1.0]
        p = [617.4435424804688, 0.0, 317.70989990234375, 0.0, 0.0, 617.878662109375, 238.01991271972656, 0.0, 0.0, 0.0, 1.0, 0.0]
        return self.get_camera_info_msg(msg_header, k, p)

    def get_depth_info(self, msg_header):
        k = [384.4228820800781, 0.0, 320.81378173828125, 0.0, 384.4228820800781, 240.3497314453125, 0.0, 0.0, 1.0]
        p = [384.4228820800781, 0.0, 320.81378173828125, 0.0, 0.0, 384.4228820800781, 240.3497314453125, 0.0, 0.0, 0.0, 1.0, 0.0]
        return self.get_camera_info_msg(msg_header, k, p)

    def find_device_that_supports_advanced_mode(self):
        ctx = rs.context()
        # ds5_dev = rs.device()
        devices = ctx.query_devices()
        for dev in devices:
            if dev.sensors[0].is_depth_sensor():
                dev.sensors[0].set_option(rs.option.enable_auto_exposure, False)
                dev.sensors[0].set_option(rs.option.exposure, 2000)
                dev.sensors[0].set_option(rs.option.depth_units, 0.001)      #default unit in mm

            if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in self.DS5_product_ids:
                if dev.supports(rs.camera_info.name):
                    print("Found device that supports advanced mode:", dev.get_info(rs.camera_info.name))
                return dev
        raise Exception("No device that supports advanced mode was found")

    def enable_advanced_mode(self):
        dev = self.find_device_that_supports_advanced_mode()
        self.advnc_mode = rs.rs400_advanced_mode(dev)
        print("Advanced mode is", "enabled" if self.advnc_mode.is_enabled() else "disabled")

        # Loop until we successfully enable advanced mode
        while not self.advnc_mode.is_enabled():
            print("Trying to enable advanced mode...")
            self.advnc_mode.toggle_advanced_mode(True)
            # At this point the device will disconnect and re-connect.
            print("Sleeping for 5 seconds...")
            time.sleep(5)
            # The 'dev' object will become invalid and we need to initialize it again
            dev = self.find_device_that_supports_advanced_mode()
            self.advnc_mode = rs.rs400_advanced_mode(dev)

        current_std_depth_control_group = self.advnc_mode.get_depth_table()
        print(current_std_depth_control_group)
        # current_std_depth_control_group.depthUnits = 100
        # current_std_depth_control_group.disparityShift = 100

        # current_std_depth_control_group.depthClampMax = 0.5
        # current_std_depth_control_group = 0.0001

    
    def readFrame(self):
        '''
        Read an aligned RGB and Depth image from the realsense camera
        
        @returns:
        colour_image - array - a numpy array HxWx3 containing the colour (rgb) image
        depth_image  - array - a numpy array HxWx1 containing the depth image
        colour_frame - pyrealsense frame - the frame from which the colour image was extracted
        depth_frame  - pyrealsense frame - the frame from which the depth image was extracted
        '''
        
        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()
        
        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)
        
        # set the depth scale so we know how to convert to meters or mm
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        # Get aligned frames
        depth_frame = aligned_frames.get_depth_frame()
        self.depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        
        colour_frame = aligned_frames.get_color_frame()
        self.colour_intrin = colour_frame.profile.as_video_stream_profile().intrinsics
        #print color_intrin
        
        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = self.profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
                
        # Validate that both frames are valid
        if not depth_frame or not colour_frame:
            print("frames not valid")
            
        # convert frames to numpy arrays
        colour_image = np.asanyarray(colour_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image = np.expand_dims(cv2.normalize(depth_image, None, 0, 255,
                                                           cv2.NORM_MINMAX, cv2.CV_8U), axis=-1)
        
        return colour_image, depth_image, colour_frame, depth_frame
                                

def main():
    '''
    Test code for checking the camera runs
    '''
    rospy.init_node('realsense_camera', anonymous=False)
    serialNum = 828612060257#rospy.get_param("%s/--serialNum"%rospy.get_name())
    
    rgb_camera_info_pub    = rospy.Publisher('/camera/color/camera_info', CameraInfo, queue_size=1)
    img_pub                = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1)
    depth_camera_info_pub  = rospy.Publisher('/camera/depth/camera_info', CameraInfo, queue_size=1)
    depth_pub              = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=1)
    
    rc = RealsenseCamera(serialNum)

    while(1):
        colour_image, depth_image, col_fr, depth_fr = rc.readFrame()
        
        ros_colour_img = rc.bridge.cv2_to_imgmsg(colour_image, "bgr8")
        ros_depth_img = rc.bridge.cv2_to_imgmsg(depth_image, "mono8")

        rgb_info_msg   = rc.get_rgb_info(ros_colour_img.header)
        depth_info_msg = rc.get_depth_info(ros_depth_img.header)

        stamp = \
        ros_colour_img.header.stamp = \
        ros_depth_img.header.stamp = \
        rgb_info_msg.header.stamp = \
        depth_info_msg.header.stamp = rospy.Time.now()
        img_pub.publish(ros_colour_img)
        rgb_camera_info_pub.publish(rgb_info_msg)
        depth_camera_info_pub.publish(depth_info_msg)
        depth_pub.publish(ros_depth_img)
        #print("Hiiiii")


if __name__ == '__main__':
    main()