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


class SwarmDriver:

    def __init__(self):

        rospy.init_node('swarm_driver_' ,anonymous=True)

        self.group = rospy.get_param('~group', 'A')
        self.rosbag_id = rospy.get_param('~rosbag_id', 1)
        self.master = rospy.get_param('~master', True)

        
        #inn the future make it a param
        self.drone_data = [
            ['0','192.168.0.101', '9010', 'A'],
            ['1','192.168.0.104', '9011', 'A'],
            ['2','192.168.0.109', '9012', 'A'],
            ['3','192.168.0.102', '9013', 'B'],
            ['4','192.168.0.103', '9014', 'B'],
            ['5','192.168.0.110', '9015', 'B'],
            ['6','192.168.0.106', '9016', 'B'],
            ['7','192.168.0.105', '9017', 'C'],
            ['8','192.168.0.111', '9018', 'C'],
            ['9','192.168.0.113', '9019', 'C']
        ]


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

        self.arm_message = ["arm","ABC"]
        self.retry_message = ["retry","ABC"]
        self.land_message = ["land","ABC"]



        self.takeoff = False
        self.bag_file_path = ""
        self.rosbag_iteration = 0
        self.rosbag_data = np.array([])
        self.sliced_data = []
        self.slicing_rate = 65 #70
        self.uuid = ""
        self.mpad_from_drones = 0
        self.known_mpad = [0]
        self.rate = rospy.Rate(20)
        self.str_data = []
        self.id_of_data = 0
        self.drone_that_has_published = []
        self.id_status = 0
        self.percentage_status = 0
        self.landed_status = 0
        self.index = 0
        self.change = 1
        self.ok = False
        self.group_counts = {}
        self.group_indices = {}
        self.group_info_tuple = []
        self.group_count_info = []
        


        self.collision_pub = rospy.Publisher('/{}/collision'.format(self.group), Int32, queue_size=10)
        self.sequence_pub = rospy.Publisher('/{}/sequence_command'.format(self.group), Array, queue_size=10)
        self.takeoff_command_pub = rospy.Publisher('/{}/takeoff_command'.format(self.group), Int32, queue_size=10)
        self.mpad_pub = rospy.Publisher('/mpad_database', Array, queue_size=10)
        self.control_pub = rospy.Publisher('/control', String_Array, queue_size=10)
        self.mpad_sub = rospy.Subscriber('/mpad', Array, self.get_mpad, queue_size=10)
        self.status_sub = rospy.Subscriber('/status', Array, self.get_status, queue_size=10)
        self.batt_sub = rospy.Subscriber('/batt_data', Array, self.get_batt, queue_size=10)

        
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

        print("Group Information Tuple:", self.group_info_tuple)
        print("Group Count Information:", self.group_count_info)

        current_group = self.group_info_tuple.index(self.group)
        self.info = self.group_count_info[current_group-1]
        self.drone_num = self.info[0]





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
                    #print(self.sliced_data)
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
        if event.button == 1:  # Left mouse button

            if 0 <= self.index < len(self.sliced_data):
                x_data = self.sliced_data[:, 0]
                y_data = self.sliced_data[:, 1]

            scatter.remove()
               
            self.sliced_data[self.index][0] = event.xdata
            self.sliced_data[self.index][1] = event.ydata
            self.change = 2



        

    def publish_takeoff_command(self): #hardcoded
        takeoff_command = Int32()
        takeoff_command.data = 1  
        self.takeoff_command_pub.publish(takeoff_command)
        self.rate = rospy.Rate(1)

    def sequencer(self):

        current_sequence = []

        time.sleep(1)

        for i in range(self.drone_num):
            for j in range(1):
                if i not in current_sequence:
                    current_sequence.append(i)

                self.sequence_pub.publish(current_sequence)
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

        for id in range(self.drone_num):

            num = self.info[id+1]
            
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



    def get_status(self, data):
        status_data = [0,0]
        status_data = data.data
        self.id_status = status_data[0]
        self.percentage_status = status_data[3]
        self.landed_status = status_data[4]

        self.status[self.id_status][0] = "Tello:{}".format(self.id_status)
        self.status[self.id_status][1] = "Group:{}".format(self.find_group(self.id_status))

        if self.landed_status == 1:
            self.status[self.id_status][2] = "Status:Landed"
            self.update_percentage(self.id_status, -1)
        else:
            self.status[self.id_status][2] = "Status:{}%".format(self.percentage_status)
            self.update_percentage(self.id_status, self.percentage_status)


        self.status[self.id_status][3] = "Batt:{}%".format(self.find_batt(self.id_status))


    

    def update_status(self):
        get_status = True
        while not rospy.is_shutdown():

            if get_status:

                color = (0, 0, 255) if not self.ok else (0, 255, 0)
                img = np.zeros((350, 600, 3), dtype=np.uint8)
                img[:, :] = color

                for i, status_row in enumerate(self.status):
                    tello_id = status_row[0]

                    # Include tello_id in the displayed status information
                    status_str = "Tello {}: {}".format(tello_id, " ".join(map(str, status_row[1:])))
                    cv2.putText(img, status_str, (10, 30 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                cv2.imshow(self.window_name, img)
                self.key = cv2.waitKey(100)  
                rospy.Rate(10).sleep()

                if self.key == ord('q'):
                    cv2.destroyAllWindows()
                    get_status = False
                elif self.key == ord('t'):
                    self.takeoff = True
                elif self.key == ord('a'):
                    self.arm()
                elif self.key == ord('r'):
                    self.retry()
                elif self.key == ord('l'):
                    self.land()

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

    def get_batt(self, data):
        batt_data = [0,0]
        batt_data = data.data
        id = batt_data[0]
        batt_level = batt_data[1]
        self.status[id][3] = "Batt:{}%".format(batt_level)

    def arm(self):
        for i in range(1):
            self.control_pub.publish(self.arm_message)
            time.sleep(0.1)

    def retry(self):
        for i in range(1):
            self.control_pub.publish(self.retry_message)
            time.sleep(0.1)

    def land(self):
        for i in range(1):
            self.control_pub.publish(self.land_message)
            time.sleep(0.1)


                    



        
        
            

    def show_status(self):
        update_thread = threading.Thread(target=self.update_status)
        update_thread.start()

    def find_group(self, id):
        group = self.drone_data[id][3]
        return group
    
    def find_batt(self, id): #update
        return 0
    
    



    def get_mpad(self, data):
        mpad_data = [0,0]
        mpad_data = data.data
        self.mpad_from_drones = mpad_data[1]
        self.id_of_data = mpad_data[0]
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
        

    def sequencer(self):

        current_sequence = []

        time.sleep(5)

        for i in range(self.drone_num):
            for j in range(11):
                if i not in current_sequence:
                    current_sequence.append(i)

                self.sequence_pub.publish(current_sequence)
                time.sleep(1)
        

    def start(self):
        if self.master:
            self.start_uwb()
            time.sleep(2)
            self.start_uwb_tf()
        self.process_drone_data()
        self.show_status()
        self.get_rosbag()
        self.visualise_data()
        self.pass_launch_args()
        time.sleep(2)
        while not rospy.is_shutdown():
                if self.takeoff:
                    sequencer_thread = threading.Thread(target=self.sequencer)
                    sequencer_thread.start()
                    while not rospy.is_shutdown():
                        self.publish_takeoff_command()
                        


if __name__ == '__main__':
    try:
        Swarm_driver = SwarmDriver()
        Swarm_driver.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Hardcode swarm driver: {}".format(e))