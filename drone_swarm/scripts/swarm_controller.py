#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import roslaunch

class SwarmController:

    def __init__(self):
        
        rospy.init_node('Swarm Controller', anonymous=True)
        
        self.rosbag_id_1 = rospy.get_param('~rosbag_id', 0)
        self.rosbag_id_2 = rospy.get_param('~rosbag_id', 0)

        self.groups = ['A','B']
        self.drone_in_groups = [5,5]

        self.takeoff_pub = rospy.Subscriber('/{}/takeoff_command'.format(self.group), Int32, queue_size=10)

    def launcher(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        
        launch_file = "swarm.launch"
        launch_files = []
        cli_args = []

        for num in range(self.groups): #only applicable till 9 drones

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



if __name__ == '__main__':
    try:
        Swarm_controller = SwarmController()
        Swarm_controller.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Swarm controller: {}".format(e))