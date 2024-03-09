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
from geometry_msgs.msg import Pose
import time
import string
import threading
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # For 3D scatter plot


class SwarmDriver:

    def __init__(self):

        rospy.init_node('swarm_driver_' ,anonymous=True)

        self.group = rospy.get_param('~group', 'A')
        self.drone_num = rospy.get_param('~drone_num', 1)
        self.rosbag_id = rospy.get_param('~rosbag_id', 1)
        self.drone_data = [
            ['0','192.168.0.100', '9010'],
            ['1','192.168.0.102', '9011'],
            ['2','192.168.0.103', '9012'],
            ['3','192.168.0.104', '9013'],
            ['4','192.168.0.105', '9014'],
        ]

        self.takeoff = 0
        self.bag_file_path = ""
        self.rosbag_iteration = 0
        self.rosbag_data = np.array([])
        self.sliced_data = []
        self.slicing_rate = 10 #70
        self.uuid = ""
        self.mpad_from_drones = 0
        self.known_mpad = [0]
        self.rate = rospy.Rate(20)
        self.data = [0,0]
        self.str_data = []
        self.id_of_data = 0
        self.drone_that_has_published = []

        self.sequence_pub = rospy.Publisher('/{}/sequence_command'.format(self.group), Array, queue_size=10)
        self.takeoff_command_pub = rospy.Publisher('/{}/takeoff_command'.format(self.group), Int32, queue_size=10)
        self.mpad_pub = rospy.Publisher('/mpad_database', Array, queue_size=10)
        self.mpad_sub = rospy.Subscriber('/mpad', Array, self.get_mpad, queue_size=10)
        
        #for uwb
        #self.rosbag_sub = rospy.Subscriber('/{}/uwb_pose'.format(self.rosbag_id), Pose, self.get_rosbag_data, queue_size=10)
       
       #for sim
        self.rosbag_sub = rospy.Subscriber('/{}/mouse_pose'.format(self.rosbag_id), Pose, self.get_rosbag_data, queue_size=10)



    def get_rosbag(self):
        self.bag_file_path = "/home/moe/catkin_ws/src/drone_swarm/rosbag/final/rosbag{}.bag".format(self.rosbag_id)
        try:
            playback_rate = 100
            command = ['rosbag', 'play', '-r', str(playback_rate), self.bag_file_path]
            process = subprocess.Popen(command)
            process.wait()
            self.process_rosbag_data()
        except Exception as e:
            rospy.logerr("Error at rosbag playback: {}".format(e))

    def get_rosbag_data(self, data):

        rosbag_x = int(data.position.x)
        rosbag_y = int(data.position.y)

        # Append new data
        new_row = np.array([[rosbag_x, rosbag_y]])

        # If it's the first row, initialize the array
        if self.rosbag_data.size == 0:
            self.rosbag_data = new_row
        else:
            # Append the new row vertically
            self.rosbag_data = np.vstack([self.rosbag_data, new_row])

        self.rosbag_iteration += 1



    def process_rosbag_data(self):

        visualise_data = True

        self.sliced_data = self.rosbag_data[::self.slicing_rate]
        self.str_data = [" ".join(map(str, row)) for row in self.sliced_data]

        if visualise_data == True:

            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')  # For 3D scatter plot

            x_data = self.sliced_data[:, 0]
            y_data = self.sliced_data[:, 1]

            x_rosbag = self.rosbag_data[:, 0]
            y_rosbag = self.rosbag_data[:, 1]

            ax.scatter(x_data, y_data, c='red', marker='o', label='Sliced Data', alpha=1)
            ax.scatter(x_rosbag, y_rosbag, c='red', marker='o', label='Rosbag Data', alpha = 0.01)

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_title('Scatter Plot of Sliced Data')

            # Show the plot
            plt.show()




    def publish_takeoff_command(self): #hardcoded
        takeoff_command = Int32()
        takeoff_command.data = 1  
        self.takeoff_command_pub.publish(takeoff_command)
        self.rate = rospy.Rate(1)

    
    def sequencer(self):
        i = 0
        a = -1
        while not rospy.is_shutdown():
            current_sequence = []
            if i == 10:
                if a != 1:
                    current_sequence.append(a + 1)
                    a += 1
                    i = 0
            elif a == 1:
                current_sequence.append(a + 1)
                a += 1
                i = 0
            self.sequence_pub.publish(current_sequence)
            i += 1
            time.sleep(1)

    def start_uwb(self):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch_file = "linktrack_rviz.launch"
        launch_files = []

        cli_args = ['nlink_parser', launch_file]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        
        launch_files=[(roslaunch_file, None)]

        parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

        parent.start()


    
    def start_uwb_tf(self):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch_file = "linktrack.launch"
        launch_files = []

        cli_args = ['nlink_parser', launch_file]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        
        launch_files=[(roslaunch_file, None)]

        parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

        parent.start()








    def pass_launch_args(self):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        for num in range(self.drone_num):
            
            launch_file = "hardcode_drone.launch"
            launch_files = []

            formatted_ip = self.drone_data[num][1]
            formatted_port = self.drone_data[num][2]


            cli_args = ['drone_swarm',
                        launch_file,
                        'name:=tello{}'.format(num),
                        'id:={}'.format(num),
                        'drone_ip:={}'.format(formatted_ip),
                        'local_port:={}'.format(formatted_port),
                        'group:={}'.format(self.group),
                        'target:={}'.format(self.pass_processed_rosbag_data()),                        
                        ]
                        
            roslaunch_args = cli_args[2:]
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
            
            launch_files=[(roslaunch_file, roslaunch_args)]

            parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

            parent.start()
            
    def pass_processed_rosbag_data(self):
        
        #hardcoded_instructions = [
        #    '148 305',
        #    '280 553',
        #    '114 101'
        #]

        #return json.dumps(hardcoded_instructions)
        #print(f"hello, this is the data {self.str_data}")
        return json.dumps(self.str_data)

    def start(self):
        #self.start_uwb()
        #time.sleep(2)
        #self.start_uwb_tf()
        self.get_rosbag()
        self.pass_launch_args()
        time.sleep(2)
        sequencer_thread = threading.Thread(target=self.sequencer)
        sequencer_thread.start()
        while not rospy.is_shutdown():
            self.publish_takeoff_command()


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