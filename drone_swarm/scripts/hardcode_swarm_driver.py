#! /usr/bin/env python3

import rospy
import random
import subprocess
import roslaunch
import numpy as np
import json
from drone_swarm.msg import Array
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
import time
import string
import threading


class SwarmDriver:



    def __init__(self):

        rospy.init_node('swarm_driver_' ,anonymous=True)

        self.group = rospy.get_param('~group', 'A')
        self.drone_num = rospy.get_param('~drone_num', 1)
        self.rosbag_id = rospy.get_param('~rosbag_id', 1)

        self.takeoff = 0
        self.sequence_delay = 5 #interval for each drone
        self.sequence_rate = rospy.Rate(1/self.sequence_delay)
        self.bag_file_path = ""
        self.rosbag_iteration = 0
        self.rosbag_data = np.array([0,0])
        self.sliced_data = []
        self.slicing_rate = 10
        self.uuid = ""
        self.mpad_from_drones = 0
        self.known_mpad = [0]
        self.rate = rospy.Rate(20)
        self.data = [0,0]
        self.id_of_data = 0
        self.drone_that_has_published = []


        self.sequence_pub = rospy.Publisher('/{}/sequence_command'.format(self.group), Array, queue_size=10)
        self.takeoff_command_pub = rospy.Publisher('/{}/takeoff_command'.format(self.group), Int32, queue_size=10)
        self.mpad_pub = rospy.Publisher('/mpad_database', Array, queue_size=10)
        self.mpad_sub = rospy.Subscriber('/mpad', Array, self.get_mpad, queue_size=10)



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
        time.sleep(5)
        while not rospy.is_shutdown():
            current_sequence = []
            current_sequence.append(0)
            self.sequence_pub.publish(current_sequence)
            self.sequence_rate.sleep()


    def pass_launch_args(self):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        for num in range(self.drone_num): #only applicable till 9 drones
            
            launch_file = "hardcode_drone.launch"
            launch_files = []


            cli_args = ['drone_swarm',
                        launch_file,
                        'name:=tello{}'.format(num),
                        'id:={}'.format(num),
                        'drone_ip:=192.168.0.10{}'.format(num + 1),
                        'local_port:=901{}'.format(num),
                        'group:={}'.format(self.group),
                        'target:={}'.format(self.pass_processed_rosbag_data()),                        
                        ]
                        
            roslaunch_args = cli_args[2:]
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
            
            launch_files=[(roslaunch_file, roslaunch_args)]

            parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

            parent.start()
            
    def pass_processed_rosbag_data(self):
        hardcoded_instructions = [
            'rc 100 0 0 0',
            'rc 100 10 0 0',
            'rc 100 10 0 0',
            'rc 100 10 0 0'
        ]
        
        return json.dumps(hardcoded_instructions)

    def start(self):
        self.send_mpad
        self.pass_launch_args()
        time.sleep(2)
        sequencer_thread = threading.Thread(target=self.sequencer)
        sequencer_thread.start()
        while not rospy.is_shutdown():
            self.publish_takeoff_command()


    def send_mpad(self):
        while not rospy.is_shutdown():
            self.mpad_pub.publish(self.known_mpad)

    def get_mpad(self, data):
        self.data = data.data
        self.mpad_from_drones = self.data[1]
        self.id_of_data = self.data[0]
        self.update_mpad()

    def update_mpad(self):
        special = None
        rescured = None
        
        if self.mpad_from_drones != -1:
            if self.mpad_from_drones not in self.known_mpad:
                self.known_mpad.append(self.mpad_from_drones)
                self.mpad_pub.publish(self.known_mpad)
                self.drone_that_has_published.append(self.id_of_data)
            elif self.mpad_from_drones in self.known_mpad and self.id_of_data not in self.drone_that_has_published:
                special = self.check_special(self.mpad_from_drones)
                if special is False:
                    self.mpad_pub.publish(self.known_mpad)
                elif special is True:
                    rescured = self.check_rescured(self.mpad_from_drones)
                    if rescured is False:
                        self.known_mpad.append(self.mpad_from_drones)
                        self.mpad_pub.publish(self.known_mpad)
                    elif rescured is True:
                        self.mpad_pub.publish(self.known_mpad)
        elif self.mpad_from_drones == -1:
            self.mpad_pub.publish(self.known_mpad)



    def check_special(self, mpad):
        if (mpad % 2 == 0) :
            return True
        else:
            return False
        
    def check_rescured(self, mpad):

        count = 0

        count = self.known_mpad.count(mpad)
        if count == 1:
            return False
        elif count == 2:
            return True
        else:
            rospy.logerr("Unexpected count value: {}".format(count))
            return False        
        



if __name__ == '__main__':
    try:
        Swarm_driver = SwarmDriver()
        Swarm_driver.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Hardcode swarm driver: {}".format(e))