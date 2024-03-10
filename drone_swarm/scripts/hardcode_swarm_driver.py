#! /usr/bin/env python3

import rospy
import random
import subprocess
import roslaunch
import numpy as np
import json
import cv2 
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
import mplcursors


class SwarmDriver:

    def __init__(self):


        rospy.init_node('swarm_driver_' ,anonymous=True)
        self.group = rospy.get_param('~group', 'A')
        self.rosbag_id = rospy.get_param('~rosbag_id', 1)


        self.visualise_data = True
        self.slicing_rate = 70 #70
        self.drone_data = [
            ['0','192.168.0.100', '9010', 'A'],
            ['1','192.168.0.102', '9011', 'A'],
            ['2','192.168.0.103', '9012', 'A'],
            ['3','192.168.0.104', '9013', 'A'],
            ['4','192.168.0.105', '9014', 'A'],
            ['5','192.168.0.106', '9015', 'B'],
            ['6','192.168.0.107', '9016', 'B'],
            ['7','192.168.0.108', '9017', 'B'],
            ['8','192.168.0.109', '9018', 'B'],
            ['9','192.168.0.110', '9019', 'B']
        ]


        self.drone_num = self.get_drone_num()
        self.takeoff = 0
        self.bag_file_path = ""
        self.rosbag_iteration = 0
        self.rosbag_data = np.array([])
        self.sliced_data = []
        self.uuid = ""
        self.mpad_from_drones = 0
        self.known_mpad = [0]
        self.rate = rospy.Rate(20)
        self.str_data = []
        self.drone_that_has_published = []
        self.index = 0
        self.change = 1
        self.ok = False


        self.sequence_pub = rospy.Publisher('/{}/sequence_command'.format(self.group), Array, queue_size=10)
        self.takeoff_command_pub = rospy.Publisher('/{}/takeoff_command'.format(self.group), Int32, queue_size=10)
        self.mpad_pub = rospy.Publisher('/mpad_database', Array, queue_size=10)
        self.mpad_sub = rospy.Subscriber('/mpad', Array, self.get_mpad, queue_size=10)
        self.status_sub = rospy.Subscriber('/status', Array, self.get_status, queue_size=10)
        
        #for uwb
        #self.rosbag_sub = rospy.Subscriber('/{}/uwb_pose'.format(self.rosbag_id), Pose, self.get_rosbag_data, queue_size=10)
       
       #for sim
        self.rosbag_sub = rospy.Subscriber('/{}/mouse_pose'.format(self.rosbag_id), Pose, self.get_rosbag_data, queue_size=10)

    def get_drone_num(self):
        drone_num = 0
        for drone in self.drone_data:
            if self.group in drone:
                drone_num += 1
        return drone_num



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

        new_row = np.array([[rosbag_x, rosbag_y]])
        if self.rosbag_data.size == 0:
            self.rosbag_data = new_row
        else:
            self.rosbag_data = np.vstack([self.rosbag_data, new_row])
        self.rosbag_iteration += 1


    def process_rosbag_data(self):
        self.sliced_data = self.rosbag_data[::self.slicing_rate]
        self.str_data = [" ".join(map(str, row)) for row in self.sliced_data]


    def visualise_data(self):
        visualize_thread = threading.Thread(target=self.start_visualising)
        visualize_thread.start()


    def start_visualising(self):
        if self.visualise_data == True:

            fig = plt.figure()
            ax = fig.add_subplot(111) 

            plt.ion() 
            plt.show()

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

                    cursor = mplcursors.cursor(hover=True)
                    cursor.connect("add", lambda sel: self.on_point_select(sel, scatter_sliced))
                    fig.canvas.mpl_connect('button_press_event', lambda event: self.on_click(event, scatter_sliced))
                    print(self.sliced_data)
                    self.str_data = [" ".join(map(str, row)) for row in self.sliced_data]
    
                plt.pause(1)
                time.sleep(0.5)


    def on_point_select(self, sel, scatter):

        index = sel.index

        if index < len(self.sliced_data):
            selected_point = self.sliced_data[index, :]
            self.index = index
            scatter.set_facecolors(['green' if i == index else 'red' for i in range(len(scatter.get_offsets()))])
            scatter.set_edgecolors(scatter.get_facecolors())        
        else:
            sel.annotation.set_text("")


    def on_click(self, event, scatter):
        if event.button == 1:
            if 0 <= self.index < len(self.sliced_data):
                x_data = self.sliced_data[:, 0]
                y_data = self.sliced_data[:, 1]
            scatter.remove()
            self.sliced_data[self.index][0] = event.xdata
            self.sliced_data[self.index][1] = event.ydata
            self.change = 2


    def sequencer(self):

        current_sequence = []

        time.sleep(3)

        for i in range(self.drone_num):
            for j in range(11):
                if i not in current_sequence:
                    current_sequence.append(i)
                self.sequence_pub.publish(current_sequence)
                time.sleep(1)


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
        self.get_rosbag()
        self.visualise_data()
        while not rospy.is_shutdown():
            if self.ok == True:
                self.pass_launch_args()
                time.sleep(2)
                sequencer_thread = threading.Thread(target=self.sequencer)
                sequencer_thread.start()


if __name__ == '__main__':
    try:
        Swarm_driver = SwarmDriver()
        Swarm_driver.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Hardcode swarm driver: {}".format(e))