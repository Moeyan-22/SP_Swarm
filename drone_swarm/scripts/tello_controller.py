#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from drone_swarm.msg import Array 



class DroneController:

    def __init__(self):

        rospy.init_node('drone_controller', anonymous=True)

        self.name = rospy.get_param('~name', 'tello')
        self.id = rospy.get_param('~id', 0)
        
        self.command_pub = rospy.Publisher('mpad', Array, queue_size=10)
        self.uwb_sub = rospy.Subscriber('mpad', Array, self.get_mpad, queue_size=10)