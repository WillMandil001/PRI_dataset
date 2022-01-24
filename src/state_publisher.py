#!/usr/bin/env python
# Author Willow Mandil || 01/02/2021

import rospy
from std_msgs.msg import Int16MultiArray
from actionlib_msgs.msg import GoalStatusArray

class status_publisher(object):
    def __init__(self):
        rospy.init_node('StatePublisher', anonymous=True)
        self.publish_status = rospy.Publisher('/status_of_motion', Int16MultiArray, queue_size=1)
        self.status = Int16MultiArray()

        rate = rospy.Rate(1000)
        self.move_state = rospy.Subscriber('/move_group/status', GoalStatusArray, self.callback)
        while not rospy.is_shutdown():
            rate.sleep()

    def callback(self, move_state):
        rate = rospy.Rate(1000)
        self.status.data = [i.status for i in move_state.status_list]
        i = 0
        while i < 199:
            i += 1
            self.publish_status.publish(self.status)
            rate.sleep()

if __name__ == '__main__':
    robot = status_publisher()
