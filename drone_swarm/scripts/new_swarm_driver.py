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
from geometry_msgs.msg import PoseStamped
import time
import string
import threading
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # For 3D scatter plot
import mplcursors
import re
import cProfile
import pstats
import socket
import math
import queue
import multiprocessing



#------------------------------Group_Driver----------------------------------------#

class Group_Driver:

    def __init__(self):

        rospy.init_node('group_driver' ,anonymous=True)

        #for params
        self.group = rospy.get_param('~group', 'A')
        self.rosbag_id = rospy.get_param('~rosbag_id', 1)
        self.drone_raw_data = rospy.get_param('~drone_data', '[]')
        self.drone_data = json.loads(self.drone_raw_data)
        self.simulation = rospy.get_param('~simulation', False)

        #for process_drone_data
        self.group_counts = {}
        self.group_indices = {}
        self.group_info_tuple = []
        self.group_count_info = []

        #for rosbag data processing
        self.rosbag_data = np.array([])
        self.sliced_data = []
        self.slicing_rate = 70 #70
        self.total_target = 0

        #for mpad detection
        self.known_mpad = [0]
        self.current_mpad = -1
        self.actioned = []
        self.land = []

        #for checking of control
        self.did_takeoff = True

        #for uwb
        self.uwb_values = []

        #for ports
        self.ip_and_ports = []
        self.binded_ports = []

        #for sequencer
        self.total_sequence_tick = 35
        self.sequence = []

        #for timing queue
        self.cmd_queue = queue.Queue()

        #for simualtion
        self.cmd_pubs = []


        self.mpad_sub = rospy.Subscriber('/mpad_database', Array, self.get_mpad, queue_size=10)
        self.control_sub = rospy.Subscriber('/control', String_Array, self.get_control, queue_size=10)
        self.status_pub = rospy.Publisher('/status', Array, queue_size=10)
        self.mpad_pub = rospy.Publisher('/mpad', Array, queue_size=10)


    def process_drone_data(self):
        for index, drone in enumerate(self.drone_data):
            drone_id, drone_ip, drone_port, group = drone

            if group in self.group_counts:
                self.group_counts[group] += 1
                self.group_indices[group].append(index)
            else:
                self.group_counts[group] = 1
                self.group_indices[group] = [index]

        num_of_groups = len(self.group_counts)
        group_names = list(self.group_counts.keys())
        self.group_info_tuple = [num_of_groups] + group_names
        #self.group_info_tuple = [2,'A','B']

        for group_name in group_names:
            num_of_drones = self.group_counts[group_name]
            drone_indices = self.group_indices[group_name]
            self.group_count_info.append([num_of_drones] + drone_indices)
            #self.group_count_info = [[2,1,2],[3,3,4,5]]

    
    def get_group_data(self):
        group = self.group_info_tuple.index(self.group) - 1
        group_index = []
        for num in range(self.group_count_info[group][0]):
            group_index.append(self.group_count_info[group][num+1])
        return group_index
        #[1,2] of current group
    
    def get_ports(self, group_data=[1,2]):
        
        script_path = '/home/moe/catkin_ws/src/drone_swarm/cleanup/kill_processes.sh'

        for _ in self.drone_data:
            self.ports.append(None)
            self.ip_and_ports.append(['',0])

        
        for num in group_data:
            ip = self.drone_data[num][1] #ip
            port = int(self.drone_data[num][2]) #ports

            self.ip_and_ports[num][0] = ip
            self.ip_and_ports[num][1] = port

            subprocess.run(['bash', script_path, str(port)])
            print("killed old pid on port {}".format(port))
            time.sleep(0.1)

            self.binded_ports[num] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.binded_ports[num].bind(('', port))

    def send(self, message = '', num=0):
        try:
            self.binded_ports[num].sendto(message.encode(), (self.ip_and_ports[num][0], 8889))
        except Exception as e:
            rospy.logerr("Error sending message: " + str(e))

    def send_command(self, command = '', num=0):
        if command.data != "mid?" and command.data != "rc 0 0 0 0" and command.data != "battery?":
            rospy.loginfo(f"executing command for drone {self.ip_and_ports[num][0]}: {command.data}")
        self.send(command.data, num)

        if self.simulation:
            self.cmd_pubs[num].publish(command.data)

    
    def get_simulation(self, group_data=[1,2]):
        for _ in self.drone_data:
            self.cmd_pubs.append = None
        for index in group_data:
            pub = rospy.Subscriber('/tello{}/cmd'.format(index), String, self.send_command, queue_size=10)
            self.cmd_pubs[index] = pub







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


    def process_rosbag_data(self):
        self.sliced_data = self.rosbag_data[::self.slicing_rate]
        self.sliced_data.append(['land'])
        self.total_target = len(self.sliced_data)




    def get_mpad(self, data):
        self.known_mpad = data.data




    def get_control(self, data):
        control_data = []
        control_data = data.data
        command = control_data[0]
        for elements in control_data:
            if command == "arm" and self.group == elements:
                print(f"group {self.group} Armed")
                self.execute_arm(self.get_group_data())
            elif command == "takeoff" and self.group == elements:
                print(f"group {self.group} taking off")
                self.execute_takeoff(self.get_group_data())
                self.did_takeoff = True
            elif command == "retry" and self.group == elements:
                print(f"group {self.group} retrying")
                self.execute_arm(self.get_group_data())


    def execute_arm(self, group_data=[1,2]):
        for num in group_data:
            self.send_command("command", num)
            time.sleep(0.1)
            self.send_command("mon", num)
            time.sleep(0.1)
            self.send_command("speed 100", num)
            time.sleep(0.1)
            self.send_command("battery?", num)

    def execute_takeoff(self, group_data=[1,2]):
        for num in group_data:
            self.send_command("takeoff", num)


        





    def subscribe_to_uwb(self, group_data = [1,2]):
        for _ in self.drone_data:
            self.uwb_values.append([0,0])
        for num in group_data:
            rospy.Subscriber('/nlt_anchorframe0_pose_node{}'.format(num), PoseStamped, self.get_uwb, callback_args=(num), queue_size=10)

    def get_uwb(self, data, num):
        x_value = data.pose.position.x
        y_value = data.pose.position.y
        self.uwb_values[num][0] = round(x_value * 100, 4)
        self.uwb_values[num][1] = round(y_value * 100, 4)







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
        
        
    def set_rescue(self):
        for _ in self.drone_data:
            self.land.append([0,0])

        
        
    def rescue_action_1(self, x, y, id):
        print("land 1") 
        self.land[id][0] = x
        self.land[id][1] = y

    def rescue_action_2(self, x, y, id):
        print("land 2") 
        self.land[id][0] = x
        self.land[id][1] = y










    def main_loop(self):

        visualise_data = True
        rate = rospy.Rate(20)
        shown_data = False
        tick = 0
        group = self.group_info_tuple.index(self.group)
        info = self.group_count_info[group-1]
        # info = [total,1,2]

        self.progress = []
        self.current_target = []
        self.offsets = []
        self.dists = []
        self.rc_commands = []
        self.mpad_landing = []
        self.landed = []
        response = ""
        percentage = 0
        battery = 0





        alpha = 0.5 #0.5
        rc_command = ""

        if visualise_data == True:
            fig = plt.figure()
            ax = fig.add_subplot(111) 
            plt.ion() 
            plt.show()

        for realtive_index in info[0]:
            self.progress.append(0)
            self.current_target.append(self.sliced_data[0])
            self.actioned.append(False)
            self.land.append([0,0])
            self.mpad_landing.append(False)
            self.landed.append(False)


        
        while not rospy.is_shutdown():

            if plt.fignum_exists(fig.number):
                if visualise_data and not shown_data:
                    shown_data = True
                    x_data = self.sliced_data[:, 0]
                    y_data = self.sliced_data[:, 1]

                    x_rosbag = self.rosbag_data[:, 0]
                    y_rosbag = self.rosbag_data[:, 1]

                    ax.scatter(x_data, y_data, c='red', marker='o', label='Sliced Data', alpha=1)
                    ax.scatter(x_rosbag, y_rosbag, c='red', marker='o', label='Rosbag Data', alpha=0.01)

                    ax.set_xlabel('X')
                    ax.set_ylabel('Y')
                    ax.set_title('Scatter Plot of Sliced Data')

            if self.did_takeoff == True and tick % 20 == 0: #1hz
                for i in range(info[0]):
                    for j in range(self.total_sequence_tick):
                        if i not in self.sequence:
                            self.sequence.append(info[i+1])

            if self.did_takeoff == True and tick % 5 == 0:
                if all('land' in target for target in self.current_target):
                    print("all drones landed")

                for num in range(info[0]):
                    if self.landed[num] == True:
                        break

                    id = info[num+1]
                    point = Point()
                    point.x = 0
                    point.y = 0


                    if self.sliced_data[self.progress[num]] == "land":
                        self.cmd_queue.put("land", id)
                        self.landed[num] = True
                        break
                        

                    self.current_target[num][0] = self.sliced_data[self.progress[num]][0]
                    self.current_target[num][1] = self.sliced_data[self.progress[num]][1]


                    if self.land[id][0] != 0:
                        self.current_target[num][0] = self.land[id][0]
                        self.current_target[num][1] = self.land[id][1]
                        self.mpad_landing[num] = True
                        

                    target_point = np.array([self.current_target[num][0], self.current_target[num][0]])
                    current_point = np.array([self.uwb_values[id][0], self.uwb_values[id][1]])

                    error_x = target_point[0] - current_point[0]
                    error_y = target_point[1] - current_point[1]

                    #print(f"error x: {error_x} error y: {error_y}")

                    max_output = 70  #70
                    control_x = np.clip(error_x, -max_output, max_output)
                    control_y = np.clip(error_y, -max_output, max_output)

                    control_x = alpha * control_x + (1 - alpha) * point.x
                    control_y = alpha * control_y + (1 - alpha) * point.y

                    dist = np.linalg.norm(target_point - current_point)

                    #print(f"{self.name} current x:{self.x_value}, current y:{self.y_value}, target:{target}, error x:{error_x}, error y:{error_y}, dist: {dist}")

                    min_dist = 22 #20

                    if dist < min_dist and self.mpad_landing[num] == True:
                        self.cmd_queue.put("land", id)
                        self.landed[num] = True
                        break

                    if dist < min_dist:
                        self.progress[num] += 1




                    point.x = control_x
                    point.y = control_y

                    if not math.isnan(point.x) and not math.isnan(point.y):
                        rc_command = "rc {} {} 0 0".format(int(point.x), int(point.y))
                    else:
                        rc_command = "rc 0 0 0 0"

                    #if self.stop_avoid == True and self.anti_collison == True:
                    #    print(f"drone:{self.id} stopped")
                    #    rc_command = "rc 0 0 0 0"
                        
                    response, _ = self.binded_ports[id].recvfrom(128)
                    response = response.decode(encoding='utf-8').strip()
                    rospy.loginfo(f"Received reply from drone: {response}")

                    if response != "ok" and response != "error":
                        if int(response) > 8:
                            battery = int(response)
                        else:
                            current_mpad = int(response)

                    special = None
                    rescured = None
                    

                    self.mpad_pub.publish([id, current_mpad])

                    if self.actioned[num] == False:
                        if current_mpad != -1:
                            if current_mpad not in self.known_mpad:
                                    print("land 1")
                                    self.mpad_pub.publish([id, current_mpad])
                                    self.actioned[num] = True
                                    self.rescue_action_1()
                            elif current_mpad in self.known_mpad:
                                special = self.check_special(current_mpad)
                                if special is False:
                                    pass
                                elif special is True:
                                    rescured = self.check_rescured(current_mpad)
                                    if rescured is False:
                                        self.actioned[num] = True
                                        self.mpad_pub.publish([id, current_mpad])
                                        print("land 2")
                                        self.rescue_action_2()
                    self.mpad_pub.publish([id, current_mpad])

                    if tick%4 == 0:
                        percentage = int((self.progress[num]/self.total_target)*100)
                        self.status_pub.publish([id, percentage, battery, self.landed[num]])
                            

                    if id in self.sequence:
                        self.cmd_queue.put(rc_command, id)

            if not self.cmd_queue.empty() and tick%20  == 0:
                cmd = self.cmd_queue.get()
                self.send_command(cmd)

            elif self.cmd_queue.empty() and tick%20 == 0:
                self.send_command("rc 0 0 0 0")

            elif tick%20 != 20 and tick%2 == 0:
                self.send_command("mid?")

            elif tick%20 != 20 and tick%3 == 0:
                self.send_command("battery?")








            #end of main loop
            tick += 1
            rate.sleep()


    def start(self):

        self.process_drone_data()
        self.get_ports(self.get_group_data())

        if self.simulation:
            self.get_simulation(self.get_group_data())

        self.get_rosbag()     
        self.subscribe_to_uwb(self.get_group_data())
        self.set_rescue()
        self.main_loop()

        









if __name__ == '__main__':

    try:
        group = Group_Driver()
        group.start()
    except rospy.ROSInterruptException:
        rospy.logerr(f"ROS process interrupted.")
    except Exception as e:
        rospy.logerr(f"Error at process: {e}")



