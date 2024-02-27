#! /usr/bin/env python3

import rospy
import subprocess
import roslaunch
import numpy as np
import json
from drone_swarm.msg import String_Array
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
import time

class SwarmDriver:

    def __init__(self):

        rospy.init_node('swarm_driver', anonymous=True)

        self.group = rospy.get_param('~group', 'A')
        self.drone_num = rospy.get_param('~drone_num', 2)
        self.rosbag_id = rospy.get_param('~rosbag_id', 1)

        self.takeoff = 0
        self.sequence_delay = 20
        self.sequence_rate = rospy.Rate(1/self.sequence_delay)
        self.bag_file_path = ""
        self.rosbag_iteration = 0
        self.rosbag_data = np.array([])
        self.sliced_data = []
        self.slicing_rate = 10
        self.uuid = ""


        self.sequence_pub = rospy.Publisher('/{}/sequence_command'.format(self.group), Int32, queue_size=10)
        self.takeoff_command_pub = rospy.Publisher('/{}/takeoff_command'.format(self.group), Int32, queue_size=10)

        """
        now triggered manually with self.takeoff = 1

        self.takeoff_sub = rospy.Subscriber('/{}/takeoff_command'.format(self.group), Int32, self.get_takeoff_command, queue_size=10)

    def get_takeoff_command(self, data):
        self.takeoff = data.data

        
        """

    def publish_takeoff_command(self): #hardcoded
        takeoff_command = Int32()
        takeoff_command.data = 1  
        self.takeoff_command_pub.publish(takeoff_command)
        self.rate = rospy.Rate(1)

    
    def sequencer(self):
        self.takeoff = 1 #manual setoff
        if self.takeoff == 1:

            for drone in self.drone_num:
                current_sequence = 1
                current_sequence += drone
                self.sequence_pub.publish(current_sequence)
                self.sequence_rate.sleep()


    def pass_launch_args(self):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        
        launch_file = "hardcode_drone.launch"
        launch_files = []
        cli_args = []

        for num in range(self.drone_num): #only applicable till 9 drones

            cli_args = ['drone_swarm',
                        launch_file,
                        '~name:=tello{}'.format(num),
                        '~id:={}'.format(num),
                        '~drone_ip:=192.168.0.10{}'.format(num + 1),
                        '~local_port:=901{}'.format(num),
                        '~group:={}'.format(self.group),
                        '~target:={}'.format(self.pass_processed_rosbag_data()),                        
                        ]
            
            roslaunch_args = cli_args[2:]
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
            
            launch_files=[(roslaunch_file, roslaunch_args)]

            parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

            parent.start()
            
    def pass_processed_rosbag_data(self):
        hardcoded_instructions = [
            'rc 0 0 0 0',
            'rc 0 0 0 0',
            'rc 0 0 0 0',
            'rc 0 0 0 0',
            'rc 0 0 0 0',
            'rc 0 0 0 0',
            'rc 0 0 0 0',
            'rc 0 0 0 0',
            'rc 0 0 0 0'
        ]
        
        return json.dumps(hardcoded_instructions)

    def start(self):
        self.pass_launch_args()
        time.sleep(5)
        print("started takeoff")
        while not rospy.is_shutdown():
            self.publish_takeoff_command()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        Swarm_driver = SwarmDriver()
        Swarm_driver.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Swarm driver: {}".format(e))