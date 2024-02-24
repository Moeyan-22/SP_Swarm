#! /usr/bin/env python3

import rospy

class SwarmController:

    def __init__(self):
        
        rospy.init_node('Swarm Controller', anonymous=True)
        
        self.rosbag_id_1 = rospy.get_param('~rosbag_id', 0)
        self.rosbag_id_2 = rospy.get_param('~rosbag_id', 0)

        self.groups = ['A','B']
        self.drone_in_groups = [5,5]


    def launcher(self):
        pass


if __name__ == '__main__':
    try:
        Swarm_controller = SwarmController()
        Swarm_controller.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Swarm controller: {}".format(e))