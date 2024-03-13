#! /usr/bin/env python3

import rospy
import roslaunch
import subprocess
import roslaunch
import numpy as np
import time
from std_msgs.msg import Int32
from drone_swarm.msg import Array
from drone_swarm.msg import String_Array
import threading
import json
import cv2 



class SwarmController:


    def __init__(self):
        
        rospy.init_node('Swarm_Controller', anonymous=True)

        self.drone_data = [
            ['0','192.168.0.100', '9010', 'A'],
            ['1','192.168.0.101', '9011', 'A'],
            ['2','192.168.0.103', '9012', 'B']
            #['3','192.168.0.104', '9013', 'C'],
            #['4','192.168.0.105', '9014', 'B'],
            #['5','192.168.0.106', '9015', 'B'],
            #['6','192.168.0.107', '9016', 'B'],
            #['7','192.168.0.108', '9017', 'B'],
            #['8','192.168.0.109', '9018', 'B'],
            #['9','192.168.0.110', '9019', 'B']
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

        self.rosbag_ids = [15,4,15]
        self.arm_message = ["arm","A","B","C"]
        self.arm_status = False

        self.window_name = "status window"
        self.mpad_from_drones = 0
        self.known_mpad = [0]
        self.drone_that_has_published = []
        self.id_of_data = 0
        self.percentage_status = 0
        self.landed_status = 0
        self.group_counts = {}
        self.group_indices = {}
        self.group_info_tuple = []
        self.group_count_info = []


    

        #make function
        self.control_pub = rospy.Publisher('/control', String_Array, queue_size=10)
        self.mpad_pub = rospy.Publisher('/mpad_database', Array, queue_size=10)
        self.mpad_sub = rospy.Subscriber('/mpad', Array, self.get_mpad, queue_size=10)
        self.status_sub = rospy.Subscriber('/status', Array, self.get_status, queue_size=10)

    def process_drone_data(self):
        for index, drone in enumerate(self.drone_data):
            drone_id, drone_ip, drone_port, group = drone

            # Update group count
            if group in self.group_counts:
                self.group_counts[group] += 1
                self.group_indices[group].append(index)
            else:
                self.group_counts[group] = 1
                self.group_indices[group] = [index]

        num_of_groups = len(self.group_counts)
        group_names = list(self.group_counts.keys())
        self.group_info_tuple = [num_of_groups] + group_names

        for group_name in group_names:
            num_of_drones = self.group_counts[group_name]
            drone_indices = self.group_indices[group_name]
            self.group_count_info.append([num_of_drones] + drone_indices)


    def pass_launch_args(self):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        for num in range(self.group_info_tuple[0]):

            
            launch_file = "hardcode_swarm.launch"
            launch_files = []

            cli_args = ['drone_swarm',
                        launch_file,
                        'group:={}'.format(self.group_info_tuple[num+1]),
                        'drone_num:={}'.format(self.group_count_info[num][0]),
                        'rosbag_id:={}'.format(self.rosbag_ids[num]),
                        'drone_data:={}'.format(self.format_drone_data()),
                        ]
                        

            roslaunch_args = cli_args[2:]
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
            
            launch_files=[(roslaunch_file, roslaunch_args)]

            parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

            parent.start()

    def format_drone_data(self):
        return json.dumps(self.drone_data)
    
    
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


    def start_sim(self):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
            
        launch_file = "sim.launch"
        launch_files = []

        cli_args = ['drone_swarm',
                    launch_file,
                    'drone_data:={}'.format(self.format_drone_data()),
                    ]
                    

        roslaunch_args = cli_args[2:]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        
        launch_files=[(roslaunch_file, roslaunch_args)]

        parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

        parent.start()

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
        else:
            self.status[self.id_status][2] = "Status:{}%".format(self.percentage_status)

        self.status[self.id_status][3] = "Batt:{}%".format(self.find_batt(self.id_status))

    def update_status(self):
        get_status = True
        while not rospy.is_shutdown():

            if get_status:

                color = (0, 0, 255) if not self.arm_status else (0, 255, 0)
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
                elif self.key == ord('a'):
                    self.arm_status = True
                    self.arm()


    def show_status(self):
        update_thread = threading.Thread(target=self.update_status)
        update_thread.start()

    def find_group(self, id):
        group = self.drone_data[id][3]
        return group
    
    def find_batt(self, id): #update
        return 0
    
    def arm(self):
        for i in range(5):
            self.control_pub.publish(self.arm_message)
            time.sleep(0.1)

    def start(self):
        #self.start_uwb()
        #self.start_uwb_tf
        self.process_drone_data()
        self.start_sim()
        self.pass_launch_args()
        self.show_status()


        
if __name__ == '__main__':
    try:
        Swarm_controller = SwarmController()
        Swarm_controller.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Swarm controller: {}".format(e))