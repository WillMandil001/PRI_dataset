#!/usr/bin/env python
# Author Willow Mandil || 01/02/2021

import os
import sys
import cv2
import copy
import time
import math
import rospy
import random
import datetime
import message_filters
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg

import numpy as np
import pandas as pd

from math import pi
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool, Int16MultiArray
from sensor_msgs.msg import JointState, Image
from moveit_msgs.msg import CollisionObject, DisplayTrajectory
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_commander.conversions import pose_to_list


class FrankaRobot(object):
    def __init__(self):
        super(FrankaRobot, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('FrankaRobotWorkshop', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.move_group.set_end_effector_link("panda_link8")
        self.group_names = self.robot.get_group_names()
        self.bridge = CvBridge()

        self.move_group.set_max_velocity_scaling_factor(0.10)  # scaling down velocity
        self.move_group.set_max_acceleration_scaling_factor(0.025)  # scaling down acceleration
        
        self.pub_start_pose = rospy.Publisher('/start_pose', PoseStamped, queue_size=1)
        self.pub_finish_pose = rospy.Publisher('/finish_pose', PoseStamped, queue_size=1)
        self.pub_joint_angle = rospy.Publisher('/joint_pose', PoseStamped, queue_size=1)
        self.stop_pub = rospy.Publisher('/stop', Bool, queue_size=1)

        self.ee_offset = 0.1

        self.bucket_height = 0.05
        self.bucket_length = 0.5
        self.bucket_width = 0.5
        self.bucket_center_x = 0.45
        self.bucket_center_y = 0.0
        self.bucket_center_z = 0.05
        self.plane_width = 0.001

        self.num_trials = 100
        self.datasave_folder = "/home/labpc/Robotics_Willow/datasets/PRI_prelim/"
        self.box_shape()

        rospy.sleep(2)
        self.add_working_window()

        self.robot_sub = message_filters.Subscriber('/joint_states', JointState)
        self.image_color_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.image_depth_sub = message_filters.Subscriber('/camera/color/depth_raw', Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.robot_sub, self.image_color_sub, self.image_depth_sub] , queue_size=1, slop=0.1, allow_headerless=True)

    def random_motion(self):
        for trial in range(self.num_trials):
            print("===== Trial ", str(trial), " out of ", self.num_trials)
            cont = raw_input("===== Continue? enter or n")
            if cont == "n":
                print("===== Finishing ")
                self.remove_objects()
                break
            else:
                if trial == 0:
                    start_position, start_ori = self.get_robot_task_state()
                finish_position = self.get_random_finish_point()
                self.orientation, orientation_eular = self.calculate_start_orientation(start_position, start_ori, finish_position)
                self.start_pose_move(start_position)
                start_position, start_ori = self.perform_point_to_point_motion(start_position, finish_position)

    def calc_angle(self,x1,x2,y1,y2):
        n1 = x2 - x1
        n2 = y2 - y1
        angle = math.atan2(n1,n2)
        if angle < 0.0:
            angle += 2*math.pi
        return angle

    def calculate_start_orientation(self, start_position, start_ori, finish_position):
        x1 = start_position[0]
        y1 = start_position[1]

        x2 = finish_position[0]
        y2 = finish_position[1]

        THETA = self.calc_angle(x1,x2,y1,y2)

        (roll, pitch, yaw) = euler_from_quaternion([start_ori[0],start_ori[1],start_ori[2],start_ori[3]])
        print(math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

        orientation_eular = [(math.pi), 0.0, - THETA + (math.pi / 2)]
        orientation = quaternion_from_euler(orientation_eular[0], orientation_eular[1], orientation_eular[2])
        return orientation, orientation_eular

    def get_random_finish_point(self):
        x = random.uniform(self.x_min, self.x_max)
        y = random.uniform(self.y_min, self.y_max)
        z = random.uniform(self.z_min, self.z_max)
        return [x,y,z]

    def box_shape(self):
        self.x_min = self.bucket_center_x - (self.bucket_length / 2)
        self.x_max = self.bucket_center_x + (self.bucket_length / 2)

        self.y_min = self.bucket_center_y - (self.bucket_width / 2)
        self.y_max = self.bucket_center_y + (self.bucket_width / 2)

        self.z_min = self.bucket_center_z + self.ee_offset
        self.z_max = self.bucket_center_z + self.bucket_height + self.ee_offset

    def get_robot_task_state(self):
        robot_ee_pose = self.move_group.get_current_pose().pose
        return [robot_ee_pose.position.x, robot_ee_pose.position.y, robot_ee_pose.position.z], [robot_ee_pose.orientation.x, robot_ee_pose.orientation.y, robot_ee_pose.orientation.z, robot_ee_pose.orientation.w]

    def start_pose_move(self, start_position):
        start_pose = PoseStamped()
        start_pose.header.frame_id = '/panda_link0'
        start_pose.pose.position.x = start_position[0]
        start_pose.pose.position.y = start_position[1]
        start_pose.pose.position.z = start_position[2]
        start_pose.pose.orientation.x = self.orientation[0]
        start_pose.pose.orientation.y = self.orientation[1]
        start_pose.pose.orientation.z = self.orientation[2]
        start_pose.pose.orientation.w = self.orientation[3]
        self.pub_start_pose.publish(start_pose)
        target = self.move_group.set_pose_target(start_pose)
        self.move_group.go(target, wait=True)
        self.move_group.stop()

    def perform_point_to_point_motion(self, start_position, finish_position):
        finish_pose = PoseStamped()
        finish_pose.header.frame_id = '/panda_link0'
        finish_pose.pose.position.x = finish_position[0]
        finish_pose.pose.position.y = finish_position[1]
        finish_pose.pose.position.z = finish_position[2]
        finish_pose.pose.orientation.x = self.orientation[0]
        finish_pose.pose.orientation.y = self.orientation[1]
        finish_pose.pose.orientation.z = self.orientation[2]
        finish_pose.pose.orientation.w = self.orientation[3]
        self.pub_finish_pose.publish(finish_pose)
        target = self.move_group.set_pose_target(finish_pose)
        trajectory = self.move_group.plan(target)
        self.move_group.go(target, wait=False)
        time_for_trajectory = trajectory.joint_trajectory.points[-1].time_from_start.secs + float("0." + str(trajectory.joint_trajectory.points[-1].time_from_start.nsecs))
        self.collect_data(time_for_trajectory)

        return [finish_pose.pose.position.x, finish_pose.pose.position.y, finish_pose.pose.position.z], [finish_pose.pose.orientation.x, finish_pose.pose.orientation.y, finish_pose.pose.orientation.z, finish_pose.pose.orientation.w]

    def collect_data(self, time_for_trajectory):

        self.robot_states = []
        self.color_images = []
        self.depth_images = []
        
        rate = rospy.Rate(10)
        self.prev_i, self.i = 0, 1
        self.ts.registerCallback(self.read_robot_data)
        t0 = time.time()
        while not rospy.is_shutdown() and time.time() - t0 < time_for_trajectory:
            print(time_for_trajectory, "    ----     ", time.time() - t0)
            self.i += 1
            rate.sleep()
        t1 = time.time()
        self.rate = (len(self.robot_states)) / (t1-t0)
        print(self.rate)
        print("FINISHED")
        save = raw_input("save the data? enter = yes, n = no")
        if save != "n":
            self.save_data()

    def read_robot_data(self, robot_joint_data, color_image, depth_image):
        if self.i != self.prev_i:
            self.prev_i = self.i
            ee_state = self.move_group.get_current_pose().pose            
            self.robot_states.append([robot_joint_data, ee_state])
            self.color_images.append(color_image)
            self.depth_images.append(depth_image)

    def format_data_for_saving(self):
        print("Formating the data")
        self.robot_states_formated = []

        for data_sample_index in range(len(self.robot_states)):
            robot_joint_data = self.robot_states[data_sample_index][0]
            ee_state = self.robot_states[data_sample_index][1]
            self.robot_states_formated.append(list(robot_joint_data.position) + list(robot_joint_data.velocity) + list(robot_joint_data.effort) + 
                                                [ee_state.position.x, ee_state.position.y, ee_state.position.z,
                                                 ee_state.orientation.x, ee_state.orientation.y, ee_state.orientation.z, ee_state.orientation.w])

    def save_data(self):
        # create new folder for this experiment:
        folder = str(self.datasave_folder + '/data_sample_' + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        mydir = os.mkdir(folder)

        self.format_data_for_saving()
        print("robot_states_formated; ", np.asarray(self.robot_states_formated).shape)
        print("rate: ", self.rate)

        T0 = pd.DataFrame(self.robot_states_formated)

        robot_states_col = ["position_panda_joint1", "position_panda_joint2", "position_panda_joint3", "position_panda_joint4", "position_panda_joint5", "position_panda_joint6", "position_panda_joint7",
        "velocity_panda_joint1", "velocity_panda_joint2", "velocity_panda_joint3", "velocity_panda_joint4", "velocity_panda_joint5", "velocity_panda_joint6", "velocity_panda_joint7",
        "effort_panda_joint1", "panda_joint2", "effort_panda_joint3", "effort_panda_joint4", "panda_joint5", "effort_panda_joint6", "effort_panda_joint7",
        "ee_state_position_x", "ee_state_position_y", "ee_state_position_z", "ee_state_orientation_x", "ee_state_orientation_y", "ee_state_orientation_z", "ee_state_orientation_w"]

        T0.to_csv(folder + '/robot_state.csv', header=robot_states_col, index=False)
        np.save(folder + '/color_images.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.color_images]))
        np.save(folder + '/depth_images.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.depth_images]))

    def perform_point_to_point_cartesian(self, start_position, finish_position):
        start_pose = PoseStamped()
        start_pose.header.frame_id = '/panda_link0'
        start_pose.pose.position.x = start_position[0]
        start_pose.pose.position.y = start_position[1]
        start_pose.pose.position.z = start_position[2]
        start_pose.pose.orientation.x = self.orientation[0]
        start_pose.pose.orientation.y = self.orientation[1]
        start_pose.pose.orientation.z = self.orientation[2]
        start_pose.pose.orientation.w = self.orientation[3]

        # Perform pushing motion
        finish_pose = PoseStamped()
        finish_pose.header.frame_id = '/panda_link0'
        finish_pose.pose.position.x = finish_position[0]
        finish_pose.pose.position.y = finish_position[1]
        finish_pose.pose.position.z = finish_position[2]
        finish_pose.pose.orientation.x = self.orientation[0]
        finish_pose.pose.orientation.y = self.orientation[1]
        finish_pose.pose.orientation.z = self.orientation[2]
        finish_pose.pose.orientation.w = self.orientation[3]

        waypoints = []
        waypoints.append(start_pose.pose)
        waypoints.append(finish_pose.pose)

        (plan, fraction) = self.move_group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.001,        # eef_step
                                           0.0)         # jump_threshold

        # display trajectory:
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory);
        
        # execute trajectory:
        self.move_group.execute(plan, wait=True)

        return [finish_pose.pose.position.x, finish_pose.pose.position.y, finish_pose.pose.position.z], [finish_pose.pose.orientation.x, finish_pose.pose.orientation.y, finish_pose.pose.orientation.z, finish_pose.pose.orientation.w]

    def remove_objects(self):
        for name in self.names:
            self.scene.remove_world_object(name)

    def wait_for_state_update(self, name, box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = name in self.scene.get_known_object_names()

            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()
        print("didnt work")
        return False

    def add_working_window(self):
        self.names = ["floor", "front", "back", "right_wall", "left_wall"]
        positions  = [[self.bucket_center_x, self.bucket_center_y, self.bucket_center_z]] #, 
                     # [((self.bucket_length/2) + self.bucket_center_x), self.bucket_center_y, self.bucket_center_z+(self.bucket_height/2)], 
                     # [((-self.bucket_length/2) + self.bucket_center_x), self.bucket_center_y, self.bucket_center_z+(self.bucket_height/2)], 
                     # [self.bucket_center_x, ((self.bucket_width/2) + self.bucket_center_y), self.bucket_center_z+(self.bucket_height/2)],
                     # [self.bucket_center_x, ((-self.bucket_width/2) + self.bucket_center_y), self.bucket_center_z+(self.bucket_height/2)]]

        shapes     = [[self.bucket_length, self.bucket_width, self.plane_width]] #,
                      # [self.plane_width, self.bucket_width, self.bucket_height],
                      # [self.plane_width, self.bucket_width, self.bucket_height],
                      # [self.bucket_length, self.plane_width, self.bucket_height],
                      # [self.bucket_length, self.plane_width, self.bucket_height]]

        for name, position, shape in zip(self.names, positions, shapes):
            scene = self.scene
            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.header.frame_id = self.planning_frame
            box_pose.pose.orientation.w = 1.0
            box_pose.pose.position.x = position[0]
            box_pose.pose.position.y = position[1]
            box_pose.pose.position.z = position[2]
            scene.add_box(name, box_pose, size=(shape[0],shape[1],shape[2]))
            self.wait_for_state_update(name, box_is_known=True, timeout=4)

if __name__ == '__main__':
    robot = FrankaRobot()
    robot.random_motion()
