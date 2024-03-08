#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

class UwbNode:


    def __init__(self):

        self.x = 0
        self.y = 0

        rospy.init_node('UWB_Echo', anonymous=True)

        self.mpad_sub = rospy.Subscriber('/nlt_anchorframe0_pose_node0', PoseStamped, self.get_coords, queue_size=10)

    def get_coords(self, data):

        x_value = data.pose.position.x
        y_value = data.pose.position.y

        self.x_value = round(x_value * 100, 4)
        self.y_value = round(y_value * 100, 4)

        print(f"x value {self.x_value}, y value {self.y_value}")



    def start(self):
        while not rospy.is_shutdown():
            pass



if __name__ == '__main__':
    try:
        uwb = UwbNode()
        uwb.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Tello driver: {}".format(e))
