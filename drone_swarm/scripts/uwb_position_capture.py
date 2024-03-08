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
from geometry_msgs.msg import PoseStamped
import subprocess
import roslaunch




class PositionCapture:

    def __init__(self):

        rospy.init_node('position_capture', anonymous=True)

        self.version = 0
        self.status = "notrecording"
        self.version_file_path = "/home/moe/catkin_ws/src/drone_swarm/rosbag/version.txt"
        self.uwb_sub = rospy.Subscriber('/nlt_anchorframe0_pose_node0', PoseStamped, self.get_coords, queue_size=10)

        self.rate = rospy.Rate(10)

    def get_coords(self, data):

        x_value = data.pose.position.x
        y_value = data.pose.position.y

        self.x_value = round(x_value * 100, 4)
        self.y_value = round(y_value * 100, 4)

        pose_msg = Pose()
        pose_msg.position.x = self.x_value
        pose_msg.position.y = self.y_value

        self.uwb_pos_pub_id.publish(pose_msg)

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
        self.uwb_pos_pub_id = rospy.Publisher('/{}/uwb_pose'.format(self.version), Pose ,queue_size=10)

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

            cv2.imshow('Keyboard Input Window', blank_image)
            self.rate.sleep()

        
    def start_rosbag(self):
        self.rosbag_path = "/home/moe/catkin_ws/src/drone_swarm/rosbag/rosbag{}".format(self.version)

        if not os.path.exists(self.rosbag_path):
            os.makedirs(self.rosbag_path)
        try:
            self.rosbag_process = subprocess.Popen(['rosbag', 'record', '-o', self.rosbag_path, '/{}/uwb_pose'.format(self.version)])
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
        self.start_uwb()
        time.sleep(1)
        self.start_uwb_tf()
        self.command_capture()

if __name__ == '__main__':
    try:
        position_capture = PositionCapture()
        position_capture.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Position controller: {}".format(e))