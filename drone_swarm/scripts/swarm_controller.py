#! /usr/bin/env python3

import rospy
import roslaunch
import time
from std_msgs.msg import Int32
import threading
import json


class SwarmController:


    def __init__(self):
        
        rospy.init_node('Swarm_Controller', anonymous=True)
        
        self.rosbag_ids_raw = rospy.get_param('~rosbag_ids', '[]')

        self.rosbag_ids = json.loads(self.rosbag_ids_raw)

        self.takeoff_pub_A = rospy.Publisher('/A/takeoff_command',Int32, queue_size=10)
        self.takeoff_pub_B = rospy.Publisher('/B/takeoff_command', Int32, queue_size=10)

        self.status = [
            #['tello','group:','action','battery:']
            ['uninitated','','uncalled',''],
            ['uninitated','','uncalled',''],
            ['uninitated','','uncalled',''],
            ['uninitated','','uncalled',''],
            ['uninitated','','uncalled',''],
            ['uninitated','','uncalled',''],
            ['uninitated','','uncalled',''],
            ['uninitated','','uncalled',''],
            ['uninitated','','uncalled',''],
            ['uninitated','','uncalled','']
        ]

        self.window_name = "status window"


    def launcher(self, agr1):        
        launch_file = "swarm.launch"
        parents = []
        launch_files = []
        cli_args = []

        for num in range(len(self.groups)):
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            if num%2 == 0 or num == 0:
                cli_args = ['drone_swarm',
                        launch_file,
                        'group:={}'.format(str(self.groups[0])),
                        'drone_num:={}'.format(int(self.drone_in_groups[0])),
                        'rosbag_id:={}'.format(int(self.rosbag_ids[0])),                      
                        ]
            else:
                cli_args = ['drone_swarm',
                            launch_file,
                            'group:={}'.format(str(self.groups[1])),
                            'drone_num:={}'.format(int(self.drone_in_groups[1])),
                            'rosbag_id:={}'.format(int(self.rosbag_ids[1])),  
                            ]
            
            roslaunch_args = cli_args[2:]
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
            launch_files=[(roslaunch_file, roslaunch_args)]
            parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
            parents.append(parent)

        for parent in parents:
            parent.start()

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
        time.sleep(0) # timing for launch for A
        thread_A.start()
        time.sleep(0) # timing for launch for B
        thread_B.start()
        thread_A.join()
        thread_B.join()

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

        while get_status:

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
            elif self.key == ord('a'):
                self.ok = True

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
        
    def start(self):
        self.start_uwb()
        time.sleep(2)
        self.start_uwb_tf()
        
if __name__ == '__main__':
    try:
        Swarm_controller = SwarmController()
        Swarm_controller.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Swarm controller: {}".format(e))