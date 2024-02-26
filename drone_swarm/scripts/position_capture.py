#! /usr/bin/env python3

import rospy
import cv2
import os
import subprocess
import signal
import numpy as np
from geometry_msgs.msg import Point


class PositionCapture:

    def __init__(self):

        rospy.init_node('position_capture', anonymous=True)

        self.version = 0
        self.status = 0
        self.version_file_path = "/home/swarm/catkin_ws/src/drone_swarm/rosbag/version.txt"
        self.rate = rospy.Rate(10)

    def check_version(self):
        try:
            with open(self.version_file_path, 'r') as file:
                version = int(file.read())
                return version
        except FileNotFoundError:
            rospy.logerr("Error at retriving version: {}".format(e))
            return 0

    def update_version(self):
        current_version = self.check_version()
        new_version = current_version + 1

        with open(self.version_file_path, 'w') as file:
            file.write(str(new_version))
        return new_version

    def command_capture(self): 
        self.version = self.check_version()
        image_height, image_width = 500, 500
        blank_image = np.zeros((image_height, image_width, 3), dtype=np.uint8)

        cv2.imshow('Keyboard Input Window', blank_image)

        while not rospy.is_shutdown():
            cv2.putText(blank_image, str(self.version), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 2, cv2.LINE_AA)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('r'):  
                if self.status == 0:
                    print("hello")
                    rospy.loginfo('Recording started for rosbag:{}')
                    self.start_rosbag()
                    blank_image[:, :] = [0, 255, 0]
                    self.status = 1
                elif self.status == 2:
                    self.version = self.update_version()
                    self.status = 0

            elif key == ord('s'): 
                rospy.loginfo('Recording stopped for rosbag:{}')
                self.stop_rosbag()
                blank_image[:, :] = [0, 0, 255]
                self.status = 2

            elif key == ord('q'):
                rospy.signal_shutdown('User pressed "q" key')

            cv2.imshow('Keyboard Input Window', blank_image)
            self.rate.sleep()

        
    def start_rosbag(self):
        self.rosbag_path = "/home/swarm/catkin_ws/src/drone_swarm/rosbag/rosbag{}".format(self.version)

        if not os.path.exists(self.rosbag_path):
            os.makedirs(self.rosbag_path)

        try:
            self.rosbag_process = subprocess.Popen(['rosbag', 'record', '-o', self.rosbag_path, '/simulated_uwb_1'])
            rospy.loginfo('ROS bag recording started.')
        except Exception as e:
            rospy.logerr('Error starting ROS bag recording: {}'.format(str(e)))


    def stop_rosbag(self):
        try:
            if self.rosbag_process:
                self.rosbag_process.send_signal(signal.SIGINT)
                self.rosbag_process.wait()
                rospy.loginfo('ROS bag recording stopped.')
            else:
                rospy.logwarn('No active ROS bag recording process to stop.')
        except subprocess.CalledProcessError as e:
            rospy.logerr('Error stopping ROS bag recording: {}'.format(str(e)))

    def start(self):
        self.command_capture()

if __name__ == '__main__':
    try:
        position_capture = PositionCapture()
        position_capture.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Position controller: {}".format(e))