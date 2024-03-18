#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from drone_swarm.msg import Array
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
import socket
import threading
import subprocess
import time


class DroneNode:

    """
    
    Single Tello Edu Driver written by Moeyan-22

    Containts Utility function for
     - executing command
     - listens for responses from drone/sim
     - checking mpad id
     - mpad logic
     - publishes actions taken for sim

    Launch Params:
     - name
     - id
     - drone ip
     - local port
    
    *Note commads such as 'mid?' and 'takeoff' is sent through tello_controller
    *added graceful shutdown

    
    """

    def __init__(self):


        rospy.init_node('drone_driver', anonymous=True)
        
        self.name = rospy.get_param('~name', 'tello')
        self.id = rospy.get_param('~id', 0)
        self.drone_ip = rospy.get_param('~drone_ip', '192.168.0.101')
        self.local_port = rospy.get_param('~local_port', 9010)

        self.response = ''
        self.current_mpad = -1
        self.known_mpad = [0]
        self.rate = rospy.Rate(20)
        self.rescue = False
        self.actioned = False
        self.x_value = 0
        self.y_value = 0



        self.mpad_pub = rospy.Publisher('/mpad', Array, queue_size=10)
        self.land_pub = rospy.Publisher('/{}/land_data'.format(self.name), Array, queue_size=10)
        self.batt_pub = rospy.Publisher('/batt_data', Array, queue_size=10)
        self.mpad_sub = rospy.Subscriber('/mpad_database', Array, self.get_mpad, queue_size=10)
        self.command_sub = rospy.Subscriber('/{}/cmd'.format(self.name), String, self.send_command, queue_size=10)
        self.uwb_sub = rospy.Subscriber('/nlt_anchorframe0_pose_node{}'.format(self.id), PoseStamped, self.get_uwb, queue_size=10)




        #kills any previously running pid
        script_path = '/home/moe/catkin_ws/src/drone_swarm/cleanup/kill_processes.sh'
        subprocess.run(['bash', script_path, str(self.local_port)])
        print("killed old pid on port {}".format(self.local_port))
        time.sleep(0.1)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', self.local_port))


    def send(self, message = ''):
        try:
            self.sock.sendto(message.encode(), (self.drone_ip, 8889))
            if message == 'land':
                time.sleep(2)
                rospy.signal_shutdown('Self-termination requested')
        except Exception as e:
            rospy.logerr("Error sending message: " + str(e))

    def send_command(self, command = ''):
        if self.rescue == False:
            #if command.data != "mid?" and command.data != "rc 0 0 0 0" and command.data != "battery?":
            rospy.loginfo(f"executing command for drone {self.drone_ip}: {command.data}")
            self.send(command.data)

    def get_uwb(self, data):
        x_value = data.pose.position.x
        y_value = data.pose.position.y
        self.x_value = round(x_value * 100, 4)
        self.y_value = round(y_value * 100, 4)
        
    def rescue_action_1(self):
        print("land 1")
        land_x = self.x_value
        land_y = self.y_value

        for i in range(10):
            self.land_pub.publish([int(land_x), int(land_y)])
            time.sleep(1)


    def rescue_action_2(self):
        print("land 2")
        land_x = self.x_value
        land_y = self.y_value

        for i in range(10):
            self.land_pub.publish([int(land_x), int(land_y)])
            time.sleep(1)
 

    def receive_command(self):
        while not rospy.is_shutdown():
            self.response, _ = self.sock.recvfrom(128)
            self.response = self.response.decode(encoding='utf-8').strip()
            #rospy.loginfo(f"Received reply from drone: {self.response}")
            if self.response != "ok" and self.response != "error":
                if int(self.response) > 8:
                    self.current_battery = int(self.response)
                    self.batt_pub.publish([self.id, self.current_battery])
                else:
                    self.current_mpad = int(self.response)
            self.rate.sleep()

    def get_mpad(self, data):
        self.known_mpad = data.data
        
    def update_mpad(self):

        special = None
        rescured = None

        self.mpad_pub.publish([self.id, self.current_mpad])

        while not rospy.is_shutdown() and self.actioned == False:
            if self.current_mpad != -1:
                if self.current_mpad not in self.known_mpad:
                        print("land 1")
                        self.mpad_pub.publish([self.id, self.current_mpad])
                        self.actioned = True
                        self.rescue_action_1()
                        time.sleep(1)
                        break
                elif self.current_mpad in self.known_mpad:
                    special = self.check_special(self.current_mpad)
                    if special is False:
                        pass
                    elif special is True:
                        rescured = self.check_rescured(self.current_mpad)
                        if rescured is False:
                            self.actioned = True
                            self.mpad_pub.publish([self.id, self.current_mpad])
                            time.sleep(0.5)
                            print("land 2")
                            self.rescue_action_2()
                            break
            self.mpad_pub.publish([self.id, self.current_mpad])
            self.rate.sleep()

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
        response_thread = threading.Thread(target=self.receive_command)
        response_thread.start()

        time.sleep(1)

        self.update_mpad()

        rospy.spin()



if __name__ == '__main__':
    try:
        drone = DroneNode()
        drone.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Tello driver: {}".format(e))
