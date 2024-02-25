#! /usr/bin/env python3

import rospy
import subprocess
import roslaunch
import numpy as np
import json
from drone_swarm.msg import Array
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Point

class SwarmDriver:

    def __init__(self):

        rospy.init_node('swarm_driver', anonymous=True)

        self.group = rospy.get_param('~group', 'A')
        self.drone_num = rospy.get_param('~drone_num', 5)
        self.rosbag_id = rospy.get_param('~rosbag_id', 0)

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
        self.takeoff_sub = rospy.Subscriber('/{}/takeoff_command'.format(self.group), Int32, self.get_takeoff_command, queue_size=10)
        self.rosbag_sub = rospy.Subscriber('/{}/rosbag'.format(self.rosbag_id), Point, self.get_rosbag_data, queue_size=10)



    def get_takeoff_command(self, data):
        self.takeoff = data.data

    def sequencer(self):
        if self.takeoff == 1:

            for drone in self.drone_num:
                current_sequence = 1
                current_sequence += drone
                self.sequence_pub.publish(current_sequence)
                self.sequence_rate.sleep()


    def get_rosbag(self):
        self.bag_file_path = "/home/swarm/catkin_ws/src/drone_swarm/rosbag/rosbag{}".format(self.rosbag_id)
        try:
            command = ['rosbag', 'play', self.bag_file_path]
            process = subprocess.Popen(command)
            process.wait()
            self.process_rosbag_data()
        except Exception as e:
            rospy.logerr("Error at rosbag playback: {}".format(e))


    def get_rosbag_data(self, data):

        rosbag_x = 0
        rosbag_y = 0

        rosbag_x = data.x
        rosbag_y = data.y
        self.rosbag_data[self.rosbag_iteration, 0] = rosbag_x
        self.rosbag_data[self.rosbag_iteration, 1] = rosbag_y
        self.rosbag_iteration += 1


    def process_rosbag_data(self):
        self.sliced_data = self.rosbag_data[::self.slicing_rate]


    def pass_launch_args(self):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        
        launch_file = "drone.launch"
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
        return json.dumps(self.sliced_data)

    def start(self):
        self.get_rosbag()
        self.pass_launch_args()

if __name__ == '__main__':
    try:
        Swarm_driver = SwarmDriver()
        Swarm_driver.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Swarm driver: {}".format(e))