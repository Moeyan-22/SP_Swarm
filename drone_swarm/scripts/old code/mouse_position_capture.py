#! /usr/bin/env python3

import rospy
import cv2
import os
import shutil
import subprocess
import signal
import numpy as np
import time
from geometry_msgs.msg import Pose, Point, Quaternion


import threading
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # For 3D scatter plot
import mplcursors
import re




class PositionCapture:

    def __init__(self):

        rospy.init_node('position_capture', anonymous=True)

        self.version = 0
        self.status = "notrecording"
        self.version_file_path = "/home/moe/catkin_ws/src/drone_swarm/rosbag/version.txt"
        self.mouse_subscriber = rospy.Subscriber('/mouse_pose', Pose, lambda data: self.get_mouse_pos(data), queue_size=10)
        self.rate = rospy.Rate(10)


        self.bag_file_path = ""
        self.rosbag_iteration = 0
        self.rosbag_data = np.array([])
        self.sliced_data = []
        self.slicing_rate = 20 #70
        self.change = 1






    def get_mouse_pos(self, data):
        self.mouse_pos_pub_id.publish(data)

    def check_version(self):
        try:
            with open(self.version_file_path, 'r') as file:
                version = int(file.read())
                return version
        except FileNotFoundError as e:
            rospy.logerr("Error at retriving version: {}".format(e))
            return 0

    def update_version(self):
        new_version = self.version + 1

        with open(self.version_file_path, 'w') as file:
            file.write(str(new_version))
        return new_version

    def command_capture(self): 
        image_height, image_width = 200, 200
        blank_image = np.zeros((image_height, image_width, 3), dtype=np.uint8)

        cv2.imshow('Keyboard Input Window', blank_image)
        self.mouse_pos_pub_id = rospy.Publisher('/{}/mouse_pose'.format(self.version), Pose ,queue_size=10)

        while not rospy.is_shutdown():
            cv2.putText(blank_image, str(self.version), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 2, cv2.LINE_AA)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('r'):  
                if self.status == "notrecording":
                    rospy.loginfo('Recording started for rosbag:{}'.format(self.version))
                    self.start_rosbag()
                    blank_image[:, :] = [0, 255, 0]
                    self.status = "recording"

            elif key == ord('s'): 
                rospy.loginfo('Recording stopped for rosbag:{}'.format(self.version))
                self.stop_rosbag()
                blank_image[:, :] = [0, 0, 255]
                self.status = "notrecording"
                self.update_version()

            elif key == ord('q'):
                rospy.signal_shutdown('User pressed "q" key')

            elif key == ord('p'):
                self.preview()

            cv2.imshow('Keyboard Input Window', blank_image)
            self.rate.sleep()







    def preview(self):
        self.rosbag_sub = rospy.Subscriber('/{}/mouse_pose'.format(self.version), Pose, self.get_rosbag_data, queue_size=10)
        self.get_rosbag()
        self.visualise_data()

    def get_rosbag(self):
        self.bag_file_path = "/home/moe/catkin_ws/src/drone_swarm/rosbag/rosbag{}.bag".format(self.version)

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

















        
    def start_rosbag(self):
        self.rosbag_path = "/home/moe/catkin_ws/src/drone_swarm/rosbag/rosbag{}".format(self.version)

        if not os.path.exists(self.rosbag_path):
            os.makedirs(self.rosbag_path)
        try:
            self.rosbag_process = subprocess.Popen(['rosbag', 'record', '-o', self.rosbag_path, '/{}/mouse_pose'.format(self.version)])
            rospy.loginfo('ROS bag recording started.')
        except Exception as e:
            rospy.logerr('Error starting ROS bag recording: {}'.format(str(e)))


    def stop_rosbag(self):  
        try:
            if self.rosbag_process:
                self.rosbag_process.send_signal(signal.SIGINT)
                self.rosbag_process.wait()
                rospy.loginfo('ROS bag recording stopped.')
                rm_rosbag_path = "/home/moe/catkin_ws/src/drone_swarm/rosbag/rosbag{}".format(self.version)
                edit_rosbag = "/home/moe/catkin_ws/src/drone_swarm/rosbag"

                if os.path.exists(rm_rosbag_path):
                    shutil.rmtree(rm_rosbag_path)

                for filename in os.listdir(edit_rosbag):
                    if "rosbag" in filename:

                        new_filename = "rosbag{}.bag".format(self.version)

                        # Build the full paths for the old and new files
                        old_path = os.path.join(edit_rosbag, filename)
                        new_path = os.path.join(edit_rosbag, new_filename)

                        # Rename the file
                        try:
                            os.rename(old_path, new_path)
                        except Exception as e:
                            print(f"Error renaming file {filename}: {str(e)}")



            else:
                rospy.logwarn('No active ROS bag recording process to stop.')
        except subprocess.CalledProcessError as e:
            rospy.logerr('Error stopping ROS bag recording: {}'.format(str(e)))

    def start(self):
        self.version = self.check_version()
        self.command_capture()

if __name__ == '__main__':
    try:
        position_capture = PositionCapture()
        position_capture.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Position controller: {}".format(e))