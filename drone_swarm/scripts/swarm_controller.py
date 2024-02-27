#! /usr/bin/env python3

import rospy
import roslaunch
import time
from std_msgs.msg import Int32
import threading


class SwarmController:


    def __init__(self):
        
        rospy.init_node('Swarm Controller', anonymous=True)
        
        self.rosbag_id_1 = rospy.get_param('~rosbag_id', 0)
        self.rosbag_id_2 = rospy.get_param('~rosbag_id', 0)

        self.rosbag_ids = [self.rosbag_id_1, self.rosbag_id_2]
        self.groups = ['A','B']
        self.drone_in_groups = [5,5]

        self.takeoff_pub_A = rospy.Publisher('/A/takeoff_command',Int32, queue_size=10)
        self.takeoff_pub_B = rospy.Publisher('/B/takeoff_command', Int32, queue_size=10)


    def launcher(self, agr1):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
    
        
        launch_file = "swarm.launch"
        launch_files = []
        cli_args = []

        for num in range(len(self.groups)): #only applicable till 9 drones
            if num/2 == 0 or num == 0:
                cli_args = ['drone_swarm',
                        launch_file,
                        '~group:={}'.format(str(self.groups[0])),
                        '~drone_num:={}'.format(int(self.drone_in_groups[0])),
                        '~rosbag_id:={}'.format(int(self.rosbag_ids[0])),                      
                        ]
            else:
                cli_args = ['drone_swarm',
                            launch_file,
                            '~group:={}'.format(str(self.groups[1])),
                            '~drone_num:={}'.format(int(self.drone_in_groups[1])),
                            '~rosbag_id:={}'.format(int(self.rosbag_ids[1])),                      
                            ]
            
            print(cli_args)
            roslaunch_args = cli_args[2:]
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
            
            launch_files=[(roslaunch_file, roslaunch_args)]

            parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

            # parent.start()

    def takeoff_A(self):   
        while not rospy.is_shutdown():
            self.takeoff_pub_A.publish(1)

    def takeoff_B(self):
        while not rospy.is_shutdown():
            self.takeoff_pub_B.publish(1)

    def start(self):
        thread_A = threading.Thread(target=self.takeoff_A)
        thread_B = threading.Thread(target=self.takeoff_B)
        self.launcher(self)
        time.sleep(10) # timing for launch for A
        thread_A.start()
        time.sleep(10) # timing for launch for B
        thread_B.start()
        thread_A.join()
        thread_B.join()
        
if __name__ == '__main__':
    try:
        Swarm_controller = SwarmController()
        Swarm_controller.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Swarm controller: {}".format(e))