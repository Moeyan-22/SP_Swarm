#! /usr/bin/env python3

import rospy
import random
import subprocess
import roslaunch
import numpy as np
import json
import cv2 
from drone_swarm.msg import Array
from drone_swarm.msg import String_Array
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
import time
import string
import threading
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # For 3D scatter plot
import mplcursors
import re
import cProfile
import pstats


class SwarmDriver:

    def __init__(self):

        rospy.init_node('swarm_driver_' ,anonymous=True)

        self.group = rospy.get_param('~group', 'A')
        self.drone_num = rospy.get_param('~drone_num', 1)
        self.rosbag_id = rospy.get_param('~rosbag_id', 1)
        self.drone_raw_data = rospy.get_param('~drone_data', '[]')
        self.drone_data = json.loads(self.drone_raw_data)

        self.status = [
            #['tello','group:','action','battery:']
            ['uninitated','','uncalled','', ''],
            ['uninitated','','uncalled','', ''],
            ['uninitated','','uncalled','', ''],
            ['uninitated','','uncalled','', ''],
            ['uninitated','','uncalled','', ''],
            ['uninitated','','uncalled','', ''],
            ['uninitated','','uncalled','', ''],
            ['uninitated','','uncalled','', ''],
            ['uninitated','','uncalled','', ''],
            ['uninitated','','uncalled','', '']
        ]

        self.window_name = "status window"

        self.percentage = [0,0,0,0,0,0,0,0,0,0]

        self.bag_file_path = ""
        self.rosbag_iteration = 0
        self.rosbag_data = np.array([])
        self.sliced_data = []
        self.slicing_rate = 70 #70
        self.uuid = ""
        self.rate = rospy.Rate(20)
        self.str_data = []

        self.id_status = 0
        self.percentage_status = 0
        self.landed_status = 0
        self.index = 0
        self.change = 1
        self.arm = False
        self.group_counts = {}
        self.group_indices = {}
        self.group_info_tuple = []
        self.group_count_info = []
        


        self.collision_pub = rospy.Publisher('/{}/collision'.format(self.group), Int32, queue_size=10)
        self.sequence_pub = rospy.Publisher('/{}/sequence_command'.format(self.group), Array, queue_size=10)
        self.takeoff_command_pub = rospy.Publisher('/{}/takeoff_command'.format(self.group), Int32, queue_size=10)
        self.status_sub = rospy.Subscriber('/status', Array, self.get_status, queue_size=10)
        self.control_sub = rospy.Subscriber('/control', String_Array, self.get_control, queue_size=10)


        
        #for uwb
        self.rosbag_sub = rospy.Subscriber('/{}/uwb_pose'.format(self.rosbag_id), Pose, self.get_rosbag_data, queue_size=10)
       
       #for sim
        #self.rosbag_sub = rospy.Subscriber('/{}/mouse_pose'.format(self.rosbag_id), Pose, self.get_rosbag_data, queue_size=10)

    def process_drone_data(self):
        # Iterate through the drone data
        for index, drone in enumerate(self.drone_data):
            drone_id, drone_ip, drone_port, group = drone

            # Update group count
            if group in self.group_counts:
                self.group_counts[group] += 1
                self.group_indices[group].append(index)
            else:
                self.group_counts[group] = 1
                self.group_indices[group] = [index]

        # Create tuples with group information
        num_of_groups = len(self.group_counts)
        group_names = list(self.group_counts.keys())
        self.group_info_tuple = [num_of_groups] + group_names

        for group_name in group_names:
            num_of_drones = self.group_counts[group_name]
            drone_indices = self.group_indices[group_name]
            self.group_count_info.append([num_of_drones] + drone_indices)


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
        self.sliced_data = self.rosbag_data[::self.slicing_rate]
        self.str_data = [" ".join(map(str, row)) for row in self.sliced_data]


    def visualise_data(self):
        visualize_thread = threading.Thread(target=self.start_visualising)
        visualize_thread.start()


    def start_visualising(self):

        visualise_data = True

        if visualise_data == True:

            fig = plt.figure()
            ax = fig.add_subplot(111) 

            plt.ion() 
            plt.show()
            rate = rospy.Rate(1)

            while not rospy.is_shutdown():
                if self.change != 0:
                    self.change = 0

                    x_data = self.sliced_data[:, 0]
                    y_data = self.sliced_data[:, 1]

                    x_rosbag = self.rosbag_data[:, 0]
                    y_rosbag = self.rosbag_data[:, 1]

                    scatter_sliced = ax.scatter(x_data, y_data, c='red', marker='o', label='Sliced Data', alpha=1)
                    ax.scatter(x_rosbag, y_rosbag, c='red', marker='o', label='Rosbag Data', alpha=0.01)

                    ax.set_xlabel('X')
                    ax.set_ylabel('Y')
                    ax.set_title('Scatter Plot of Sliced Data')

                plt.pause(1)
                rate.sleep()
                
                if not plt.fignum_exists(fig.number):
                    break






    def publish_takeoff_command(self): #hardcoded
        takeoff_command = Int32()
        takeoff_command.data = 1  
        self.takeoff_command_pub.publish(takeoff_command)

    def sequencer(self):

        current_sequence = []
        group = self.group_info_tuple.index(self.group)
        info = self.group_count_info[group-1]

        time.sleep(5)

        for i in range(info[0]):
            for j in range(35):
                if i not in current_sequence:
                    current_sequence.append(info[i+1])
                self.sequence_pub.publish(current_sequence)
                time.sleep(1)


    def pass_launch_args(self):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        group = self.group_info_tuple.index(self.group)

        for num in range(self.group_count_info[group-1][0]):
            
            launch_file = "hardcode_drone.launch"
            launch_files = []

            info = self.group_count_info[group-1]
            formatted_ip = self.drone_data[info[num+1]][1]
            formatted_port = self.drone_data[info[num+1]][2]

            cli_args = ['drone_swarm',
                        launch_file,
                        'name:=tello{}'.format(info[num+1]),
                        'id:={}'.format(info[num+1]),
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


    def get_status(self, data):
        status_data = [0,0]
        status_data = data.data
        self.id_status = status_data[0]
        self.percentage_status = status_data[3]
        self.landed_status = status_data[4]

        if self.landed_status == 1:
            self.status[self.id_status][2] = "Status:Landed"
            self.update_percentage(self.id_status, -1)
        else:
            self.status[self.id_status][2] = "Status:{}%".format(self.percentage_status)
            self.update_percentage(self.id_status, self.percentage_status)

    def update_percentage(self, id, percentage_num):
        self.percentage[id] = percentage_num
        self.check_percentage()

    def check_percentage(self):
        i = self.group_info_tuple.index(self.group) - 1
        for a in range(self.group_count_info[i][0] - 1):
            infront = self.group_count_info[i][a+1]
            infront_percentage = self.percentage[infront]
            behind = self.group_count_info[i][a+2]
            behind_percentage = self.percentage[behind]
            #print(f"group:{self.group_info_tuple[i+1]}, drone:{infront}, infront:{infront_percentage}, behind:{behind_percentage}")
            if infront_percentage != -1 and infront_percentage != 100 and infront_percentage != 0:
                if behind_percentage == infront_percentage:
                    #print(f"tello {behind} is blocked by {infront}")
                    self.status[behind][4] = "blocked"
                    self.collision_pub.publish(behind)
                else:
                    self.status[behind][4] = ""
                    self.collision_pub.publish(-1)
            else:
                self.status[behind][4] = ""
                self.collision_pub.publish(-1)

    def get_control(self, data):
        control_data = []
        control_data = data.data
        command = control_data[0]
        for elements in control_data:
            if command == "arm" and self.group == elements:
                self.arm = True


    def start(self):

        self.rate = rospy.Rate(1)
        self.process_drone_data()
        self.get_rosbag()
        self.visualise_data()
        while not rospy.is_shutdown():
            if self.arm == True:
                self.pass_launch_args()
                time.sleep(2)
                sequencer_thread = threading.Thread(target=self.sequencer)
                sequencer_thread.start()
                for i in range(1000):
                    self.publish_takeoff_command()
                    time.sleep(1)
                break
            self.rate.sleep()



if __name__ == '__main__':
    try:
        Swarm_driver = SwarmDriver()
        Swarm_driver.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Hardcode swarm driver: {}".format(e))