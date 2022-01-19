#!/usr/bin/env python
# Author Willow Mandil || 01/02/2021

import sys
import copy
import math
import rospy
import random
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import CollisionObject


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
        self.move_group.set_end_effector_link("panda_hand")
        self.group_names = self.robot.get_group_names()

        self.move_group.set_max_velocity_scaling_factor(0.10)  # scaling down velocity
        self.move_group.set_max_acceleration_scaling_factor(0.025)  # scaling down velocity
        
        self.num_trials = 100
        self.box_shape()

        self.pub_start_pose = rospy.Publisher('/start_pose', PoseStamped, queue_size=1)
        self.pub_finish_pose = rospy.Publisher('/finish_pose', PoseStamped, queue_size=1)
        self.pub_joint_angle = rospy.Publisher('/joint_pose', PoseStamped, queue_size=1)

        self.xela_sub = message_filters.Subscriber('/xServTopic', XStream)
        self.robot_sub = message_filters.Subscriber('/joint_states', JointState)
        subscribers = [self.robot_sub, self.xela_sub, self.marker_sub]
        self.ts = message_filters.ApproximateTimeSynchronizer(subscribers, queue_size=1, slop=0.01, allow_headerless=True)
        self.ts.registerCallback(self.read_robot_data)

        rospy.sleep(2)
        self.add_working_window()
        # rospy.sleep(5)
        # self.remove_objects()

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

                t0 = time.time()
                rate = rospy.Rate(1000)
                self.prev_i = 0
                self.i = 1
                start_position, start_ori = self.perform_point_to_point_motion(start_position, finish_position)
                while not rospy.is_shutdown() and self.stop is False:
                    self.i += 1
                    rate.sleep()
                t1 = time.time()
                self.rate = (len(self.xelaSensor1)) / (t1-t0) 

                self.save_data()

                # start_position, start_ori           = self.perform_point_to_point_cartesian(start_position, finish_position)

    def read_robot_data(self, robot_joint_data, xela_data, marker_data):
        if self.stop == False and self.i != self.prev_i:
            self.prev_i = self.i
            ee_state = self.group.get_current_pose().pose
            self.robot_states.append([robot_joint_data, ee_state])
            self.xelaSensor1.append(xela_data.data[0])

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
        self.x_min = 0.30
        self.x_max = 0.64

        self.y_min = -0.16
        self.y_max = 0.26

        self.z_min = 0.27
        self.z_max = 0.37

    def get_robot_task_state(self):
        robot_ee_pose = self.move_group.get_current_pose().pose
        return [robot_ee_pose.position.x, robot_ee_pose.position.y, robot_ee_pose.position.z], [robot_ee_pose.orientation.x, robot_ee_pose.orientation.y, robot_ee_pose.orientation.z, robot_ee_pose.orientation.w]

    def get_robot_joint_state(self):
        robot_state = self.robot.get_current_state().joint_state.position
        return robot_state

    def start_pose_move(self, start_position):
        # Move to the new orientation
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
        self.move_group.go(target)

    def perform_point_to_point_motion(self, start_position, finish_position):
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
        self.pub_finish_pose.publish(finish_pose)
        target = self.move_group.set_pose_target(finish_pose)
        self.move_group.go(target)

        return [finish_pose.pose.position.x, finish_pose.pose.position.y, finish_pose.pose.position.z], [finish_pose.pose.orientation.x, finish_pose.pose.orientation.y, finish_pose.pose.orientation.z, finish_pose.pose.orientation.w]

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
        
        perform_push = raw_input("press enter to execute")
        if perform_push
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
        bucket_height = 0.05
        bucket_length = 0.5
        bucket_width = 0.5
        bucket_center_x = 0.45
        bucket_center_y = 0.0
        bucket_center_z = 0.05
        plane_width = 0.001

        self.names = ["floor", "front", "back", "right_wall", "left_wall"]
        positions  = [[bucket_center_x, bucket_center_y, bucket_center_z]] #, 
                     # [((bucket_length/2) + bucket_center_x), bucket_center_y, bucket_center_z+(bucket_height/2)], 
                     # [((-bucket_length/2) + bucket_center_x), bucket_center_y, bucket_center_z+(bucket_height/2)], 
                     # [bucket_center_x, ((bucket_width/2) + bucket_center_y), bucket_center_z+(bucket_height/2)],
                     # [bucket_center_x, ((-bucket_width/2) + bucket_center_y), bucket_center_z+(bucket_height/2)]]

        shapes     = [[bucket_length, bucket_width, plane_width]] #,
                      # [plane_width, bucket_width, bucket_height],
                      # [plane_width, bucket_width, bucket_height],
                      # [bucket_length, plane_width, bucket_height],
                      # [bucket_length, plane_width, bucket_height]]

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




    def format_data_for_saving(self):
        print("Formating the data")

        self.robot_states_formated = []
        self.xelaSensor1Formatted = []

        for data_sample_index in range(len(self.xelaSensor1)):
            robot_joint_data = self.robot_states[data_sample_index][0]
            ee_state = self.robot_states[data_sample_index][1]
            self.robot_states_formated.append(list(robot_joint_data.position) + list(robot_joint_data.velocity) + list(robot_joint_data.effort) + 
                                                [ee_state.position.x, ee_state.position.y, ee_state.position.z,
                                                 ee_state.orientation.x, ee_state.orientation.y, ee_state.orientation.z, ee_state.orientation.w])

            Sensor1_data = self.xelaSensor1[data_sample_index]
            Sensor1_vector = np.zeros((1, 48))

            xela_vector = np.zeros((1, 96))
            for index, i in enumerate(range(0, 48, 3)):
                Sensor1_vector[0, i]     = Sensor1_data.xyz[index].x
                Sensor1_vector[0, i + 1] = Sensor1_data.xyz[index].y
                Sensor1_vector[0, i + 2] = Sensor1_data.xyz[index].z

            self.xelaSensor1Formatted.append(Sensor1_vector)

    def save_data(self):
        self.format_data_for_saving()

        self.xelaSensor1Formatted = np.array(self.xelaSensor1Formatted)
        self.xelaSensor1Formatted = np.reshape(self.xelaSensor1Formatted, (self.xelaSensor1Formatted.shape[0], 48))

        downsample_ratio = self.xelaSensor1Formatted.shape[0]

        print("robot_states_formated; ", np.asarray(self.robot_states_formated).shape)
        print("xelaSensor1Formatted length: ", self.xelaSensor1Formatted.shape)
        print("rate: ", self.rate)

        T1 = pd.DataFrame(self.xelaSensor1Formatted)
        T2 = pd.DataFrame(self.robot_states_formated)

        # create new folder for this experiment:
        folder = str('/home/kiyanoush/will_data_collection/data_collection_003/data_sample_' + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        mydir = os.mkdir(folder)

        xela_Sensor_col = ['txl1_x', 'txl1_y', 'txl1_z', 'txl2_x', 'txl2_y', 'txl2_z','txl3_x', 'txl3_y', 'txl3_z','txl4_x', 'txl4_y', 'txl4_z','txl5_x', 'txl5_y', 'txl5_z','txl6_x', 'txl6_y', 'txl6_z',
        'txl7_x', 'txl7_y', 'txl7_z','txl8_x', 'txl8_y', 'txl8_z','txl9_x', 'txl9_y', 'txl9_z','txl10_x', 'txl10_y', 'txl10_z','txl11_x', 'txl11_y', 'txl11_z','txl12_x', 'txl12_y', 'txl12_z',
        'txl13_x', 'txl13_y', 'txl13_z','txl14_x', 'txl14_y', 'txl14_z','txl15_x', 'txl15_y', 'txl15_z','txl16_x', 'txl16_y', 'txl16_z']

        robot_states_col = ["position_panda_joint1", "position_panda_joint2", "position_panda_joint3", "position_panda_joint4", "position_panda_joint5", "position_panda_joint6", "position_panda_joint7", "position_panda_finger_joint1", "position_panda_finger_joint2",
        "velocity_panda_joint1", "velocity_panda_joint2", "velocity_panda_joint3", "velocity_panda_joint4", "velocity_panda_joint5", "velocity_panda_joint6", "velocity_panda_joint7", "velocity_panda_finger_joint1", "velocity_panda_finger_joint2",
        "effort_panda_joint1", "panda_joint2", "effort_panda_joint3", "effort_panda_joint4", "panda_joint5", "effort_panda_joint6", "effort_panda_joint7", "effort_panda_finger_joint1", "effort_panda_finger_joint2",
        "ee_state_position_x", "ee_state_position_y", "ee_state_position_z", "ee_state_orientation_x", "ee_state_orientation_y", "ee_state_orientation_z", "ee_state_orientation_w"]

        marker_col = ["position_x", "position_y", "position_z", "quaternion_x", "quaternion_y", "quaternion_z", "quaternion_w"]

        T1.to_csv(folder + '/xela_sensor1.csv', header=xela_Sensor_col, index=False)
        T2.to_csv(folder + '/robot_state.csv', header=robot_states_col, index=False)


if __name__ == '__main__':
    robot = FrankaRobot()
    robot.random_motion()
