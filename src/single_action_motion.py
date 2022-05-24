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
from xela_server.msg import XStream
from sensor_msgs.msg import JointState, Image
from moveit_msgs.msg import CollisionObject, DisplayTrajectory, Constraints, OrientationConstraint, RobotTrajectory
from geometry_msgs.msg import PoseStamped, Quaternion
from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
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
        self.move_group.allow_replanning(True)
        self.move_group.set_num_planning_attempts(50)  # was 10
        self.move_group.set_goal_position_tolerance(0.05)
        self.move_group.set_goal_orientation_tolerance(0.1)

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

        # self.start_joint_pose  = [-1.5582958590490317, 1.5338252153731227, 1.4436745717079291, -1.9266593425650345, -1.585236271994511, 1.7241992764896417, 1.163573228574489]
        self.start_joint_pose  = [-1.5581101927305572, 1.5336645008388319, 1.446202119070332, -1.9271189652325813, -1.5845098768066816, 1.7244407136175368, 1.1638548101548194]
        self.intermediate_joint_pose = [-1.5075874511275376, 1.5628152434993339, 1.7488304869911, -2.2568651993567483, -1.4742872962752978, 1.4788165258321568, 1.625300617218406]
        self.finish_task_pose = [0.5482443163500694, 0.12686563149671773, 0.17835713843348572, -0.9189951780140059, 0.3934372105104041, -0.017203839819053317, 0.018948669300987524]

        start_position, start_ori = self.get_robot_task_state()
        print(start_position, start_ori)

        self.generate_single_motion()

        self.num_trials = 100
        self.object_number = 9
        self.datasave_folder = "/vol/hd/PRI_DATASETS/Dataset3_MarkedHeavyBox/test/"

        self.box_shape()
        rospy.sleep(2)
        self.add_working_window()

        self.robot_sub = message_filters.Subscriber('/joint_states', JointState)
        self.tactile_sub = message_filters.Subscriber('/xServTopic', XStream)
        self.image_color_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.image_depth_sub = message_filters.Subscriber('/camera/color/depth_raw', Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.robot_sub, self.image_color_sub, self.image_depth_sub, self.tactile_sub] , queue_size=1, slop=0.1, allow_headerless=True)

    def random_motion(self):
        for trial in range(self.num_trials):
            print("===== Trial ", str(trial), " out of ", self.num_trials)
            cont = raw_input("===== Continue? enter or n")
            if cont == "n":
                print("===== Finishing ")
                self.remove_objects()
                break
            else:
                # move to intermediate position.
                self.move_to_joint_state(self.intermediate_joint_pose)
                # move to start joint state.
                self.move_to_joint_state(self.start_joint_pose)
                # move to goal ee state and collect data
                self.move_and_collect_data(self.finish_task_pose)

    def move_to_joint_state(self, joint_pose):
        self.move_group.go(joint_pose, wait=True)
        self.move_group.stop()

    def move_to_task_space(self, task_space):
        start_pose = PoseStamped()
        start_pose.header.frame_id = '/panda_link0'
        start_pose.pose.position.x = task_space[0]
        start_pose.pose.position.y = task_space[1]
        start_pose.pose.position.z = task_space[2]
        start_pose.pose.orientation.x = 1
        start_pose.pose.orientation.y = 0
        start_pose.pose.orientation.z = 0
        start_pose.pose.orientation.w = 0
        self.pub_start_pose.publish(start_pose)
        target = self.move_group.set_pose_target(start_pose)
        self.move_group.go(target, wait=True)
        self.move_group.stop()

    def move_and_collect_data(self, finish_position):
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(self.robot_trajectory)
        self.display_trajectory_publisher.publish(display_trajectory)

        print(self.joint_trajectory)
        time_for_trajectory = self.robot_trajectory.joint_trajectory.points[-1].time_from_start.secs + float("0." + str(self.robot_trajectory.joint_trajectory.points[-1].time_from_start.nsecs))
        self.move_group.execute(self.robot_trajectory, wait=False)
        self.collect_data(time_for_trajectory)

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

    def collect_data(self, time_for_trajectory):
        self.robot_states = []
        self.color_images = []
        self.depth_images = []
        self.tactile_states = []
        
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

    def read_robot_data(self, robot_joint_data, color_image, depth_image, tactile_state):
        if self.i != self.prev_i:
            self.prev_i = self.i
            ee_state = self.move_group.get_current_pose().pose
            self.robot_states.append([robot_joint_data, ee_state])
            self.color_images.append(color_image)
            self.depth_images.append(depth_image)
            self.tactile_states.append(tactile_state.data[0])

    def format_data_for_saving(self):
        print("Formating the data")
        self.robot_states_formated = []
        self.tactile_states_formated = []

        for data_sample_index in range(len(self.robot_states)):
            robot_joint_data = self.robot_states[data_sample_index][0]
            ee_state = self.robot_states[data_sample_index][1]
            self.robot_states_formated.append(list(robot_joint_data.position) + list(robot_joint_data.velocity) + list(robot_joint_data.effort) + 
                                                [ee_state.position.x, ee_state.position.y, ee_state.position.z,
                                                 ee_state.orientation.x, ee_state.orientation.y, ee_state.orientation.z, ee_state.orientation.w])

            tactile_data = self.tactile_states[data_sample_index]
            tactile_vector = np.zeros(48)
            for index, i in enumerate(range(0, 48, 3)):
                tactile_vector[i]     = int(tactile_data.xyz[index].x)
                tactile_vector[i + 1] = int(tactile_data.xyz[index].y)
                tactile_vector[i + 2] = int(tactile_data.xyz[index].z)

            self.tactile_states_formated.append(tactile_vector)

    def save_data(self):
        # create new folder for this experiment:
        folder = str(self.datasave_folder + 'data_sample_' + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        mydir = os.mkdir(folder)

        self.format_data_for_saving()
        print("robot_states_formated; ", np.asarray(self.robot_states_formated).shape)
        print("tactile_states_formated; ", np.asarray(self.tactile_states_formated).shape)
        print("rate: ", self.rate)

        T0 = pd.DataFrame(self.robot_states_formated)

        robot_states_col = ["position_panda_joint1", "position_panda_joint2", "position_panda_joint3", "position_panda_joint4", "position_panda_joint5", "position_panda_joint6", "position_panda_joint7",
        "velocity_panda_joint1", "velocity_panda_joint2", "velocity_panda_joint3", "velocity_panda_joint4", "velocity_panda_joint5", "velocity_panda_joint6", "velocity_panda_joint7",
        "effort_panda_joint1", "panda_joint2", "effort_panda_joint3", "effort_panda_joint4", "panda_joint5", "effort_panda_joint6", "effort_panda_joint7",
        "ee_state_position_x", "ee_state_position_y", "ee_state_position_z", "ee_state_orientation_x", "ee_state_orientation_y", "ee_state_orientation_z", "ee_state_orientation_w"]

        print("\n \n")
        print(T0)
        print(robot_states_col)
        print("\n \n")

        T0.to_csv(folder + '/robot_states.csv', header=robot_states_col, index=False)
        np.save(folder + '/color_images.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.color_images]))
        np.save(folder + '/depth_images.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.depth_images]))
        np.save(folder + '/tactile_states.npy', np.array(self.tactile_states_formated))
        np.save(folder + '/meta.npy', np.array(["Object:", self.object_number]))

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
        positions  = [[self.bucket_center_x, self.bucket_center_y, self.bucket_center_z]]
        shapes     = [[self.bucket_length, self.bucket_width, self.plane_width]]

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

    def generate_single_motion(self):
        self.robot_trajectory = RobotTrajectory()

        self.joint_trajectory = JointTrajectory()
           
        self.joint_trajectory.header.seq = 0
        self.joint_trajectory.header.stamp.secs = 0
        self.joint_trajectory.header.stamp.nsecs = 0
        self.joint_trajectory.header.frame_id = "/panda_link0"
        self.joint_trajectory.joint_names =  ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]

        positions = [[-1.5581101927305572, 1.5336645008388319, 1.446202119070332, -1.9271189652325813, -1.5845098768066816, 1.7244407136175368, 1.1638548101548194],
                    [-1.5327034099160541, 1.5320093742880303, 1.4459221670372198, -1.9805834979262105, -1.591915536560678, 1.7241316756997636, 1.2432612028483858],
                    [-1.4983477996088739, 1.5312266297149175, 1.4458473013851156, -2.0243406376190136, -1.5987318229512903, 1.7230849387639542, 1.321912778028671],
                    [-1.455281870098177, 1.5313293963631038, 1.4460121421959822, -2.058478005396736, -1.6048823440452824, 1.7213243656523, 1.3996472354603176],
                    [-1.4038212662416796, 1.532321091170952, 1.4464571167737863, -2.0829982974893615, -1.610293896602556, 1.7188868962607904, 1.476141204248687],
                    [-1.3443907726379984, 1.5341891779980563, 1.4472246454860234, -2.097867890032615, -1.6149051783706267, 1.7158256954417608, 1.550927298321168],
                    [-1.277534963167425, 1.5369010595299257, 1.4483551253495608, -2.1030515629774778, -1.618674640588295, 1.7122110696802029, 1.62341856749887],
                    [-1.2039060786170215, 1.5404028117436375, 1.4498831775437913, -2.098533296219669, -1.6215863880638877, 1.7081291778627172, 1.6929416466047043],
                    [-1.124231211056592, 1.5446208057099646, 1.4518348306857651, -2.084323117965711, -1.6236529491296265, 1.7036787784238527, 1.7587764365735141],
                    [-1.0392642646117005, 1.5494655313271166, 1.4542262143268931, -2.060450456445197, -1.6249140643565603, 1.6989665859835312, 1.8201972503672554]]

        velocities = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.09473601709694132, -0.003515361943120175, -0.0004895971462012384, -0.14726966337313863, -0.021777547869263114, -0.002300067424849366, 0.24366372373611198],
                      [0.14223235993421257, -0.001238840846482298, 0.00016776664549375954, -0.14290333135752967, -0.02379865661525553, -0.005161639670062105, 0.2871],
                      [0.17610054396996255, 0.002050809093047385, 0.0011394543547754358, -0.10905573577784267, -0.021513411309397473, -0.007825400277942273, 0.2871],
                      [0.21064723983501682, 0.005446777877342358, 0.0023082993281331923, -0.07455703834619944, -0.01900665759094272, -0.010450097139893236, 0.2871],
                      [0.23370031657610227, 0.008438110588552046, 0.0034960177165889623, -0.03781694186303059, -0.015595930847970459, -0.012343541364682135, 0.2732585196343257],
                      [0.23925000000000002, 0.010541669589283636, 0.00450539547019599, -0.0019343009713772017, -0.011475415220118516, -0.013099502496874844, 0.24266280909313348],
                      [0.23925000000000002, 0.012022261241666759, 0.005412869830395323, 0.028676204705322634, -0.007833486817661946, -0.013313752189962485, 0.21179959653190344],
                      [0.22196059985767805, 0.012168025130569766, 0.005810462420278945, 0.05008799699201356, -0.004621672812459329, -0.012357336915879712, 0.17282155414707973],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

        accelerations = [[0.32308437735678475, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                        [0.18296770085545946, 0.003925999055298836, 0.0012904174786698504, -0.0743050732161356, -0.01851532059888477, -0.009072259262684339, 0.25911499654987197], 
                        [0.12355127614645503, 0.011884455691657372, 0.003238769745357055, 0.12353253434318415, 0.00795047018587016, -0.009845695305926048, 0.0], 
                        [0.12690610366501848, 0.012444340918368364, 0.003951202960794032, 0.12677180399541235, 0.008954466133951093, -0.009851189512382572, 0.0], 
                        [0.13287208178877766, 0.013092666668977295, 0.004844747808963176, 0.13264463229151185, 0.009900331362047064, -0.009881463512806695, 0.0], 
                        [0.04111431785462905, 0.00938364159660944, 0.004071024867729466, 0.142735313488946, 0.015606141546927655, -0.004383907411007374, -0.10254333082899576], 
                        [0.0, 0.0057013334814638465, 0.003132650687826087, 0.11319018560999891, 0.013719432345207436, -0.0011186440342092333, -0.1141319941628409], 
                        [0.0, 0.004018010734860034, 0.0027942139931918923, 0.08736097555734298, 0.010162538128007674, -0.0003123510726989717, -0.08807544471165235], 
                        [-0.09243715329877475, -0.002661947267850858, -0.0002674202951839232, 0.03965612414279478, 0.008468032156934742, 0.005380956158745796, -0.13296153091552945], 
                        [-0.9860375533229908, -0.0562228206842307, -0.02775189851130938, -0.27704115237638544, 0.014635184914296363, 0.054684779188284814, -0.712785753640897]]

        times_from_start = [[0,0],
                            [0,396580957],
                            [0,670532803],
                            [0,941290231],
                            [1,207726904],
                            [1,468214866],
                            [1,747653987],
                            [2,55402720],
                            [2,388422020],
                            [2,803560773]]

        points = []
        for pos, vel, accel, tos in zip(positions, velocities, accelerations, times_from_start):
            point = JointTrajectoryPoint()
            point.positions = pos
            point.velocities = vel
            point.accelerations = accel
            point.effort = []
            point.time_from_start.secs = tos[0]
            point.time_from_start.nsecs = tos[1]
            points.append(point)

        self.joint_trajectory.points = points

        self.robot_trajectory.joint_trajectory = self.joint_trajectory


if __name__ == '__main__':
    robot = FrankaRobot()
    robot.random_motion()
