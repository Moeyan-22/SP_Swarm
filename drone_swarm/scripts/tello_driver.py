#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from drone_swarm.msg import Array



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


        rospy.init_node('drone', anonymous=True)
        
        self.name = rospy.get_param('~name', 'tello')
        self.id = rospy.get_param('~id', 0)
        self.drone_ip = rospy.get_param('~drone_ip', '192.168.0.101')
        self.local_port = rospy.get_param('~local_port', 9010)

        self.response = ''
        self.current_mpad = -2
        self.known_mpad = [0]
        self.rate = rospy.Rate(20)
        self.rescue = False

        self.action_pub = rospy.Publisher('/{}/action'.format(self.name), String, queue_size=10)
        self.mpad_pub = rospy.Publisher('/mpad', Array, queue_size=10)
        self.mpad_sub = rospy.Subscriber('/mpad', Array, self.get_mpad, queue_size=10)
        self.command_sub = rospy.Subscriber('/{}/cmd'.format(self.name), String, self.send_command, queue_size=10)


        #kills any previously running pid
        script_path = '/home/swarm/catkin_ws/src/drone_swarm/cleanup/kill_processes.sh'
        subprocess.run(['bash', script_path, str(self.local_port)])
        print("killed old pid on port {}".format(self.local_port))

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', self.local_port))


    def send(self, message = ''):
        try:
            self.sock.sendto(message.encode(), (self.drone_ip, 8889))
        except Exception as e:
            rospy.logerr("Error sending message: " + str(e))

    def send_command(self, command = ''):
        if self.rescue == False:
            if command.data != "mid?":
                rospy.loginfo(f"executing command: {command.data}")
            self.send(command.data)
            self.action_pub.publish(command)
        else:
            self.rescue_action()

    def rescue_action(self):
        self.send('stop')
        self.send('go 100 0 0 10 {}'.format(self.current_mpad))
 

    def receive_command(self):
        while not rospy.is_shutdown():
            self.response, _ = self.sock.recvfrom(128)
            self.response = self.response.decode(encoding='utf-8').strip()
            #rospy.loginfo(f"Received reply from drone: {self.response}")
            if self.response != "-1" and self.response != "ok":
                print("is integer")
                if int(self.response) != -1:
                    self.current_mpad = int(self.response)
            self.rate.sleep()

    def get_mpad(self, data):
        self.known_mpad = data
        
    def update_mpad(self):
        
        while not rospy.is_shutdown():
            print(self.current_mpad)
            if self.current_mpad != -1:
                print("should happen")
                if self.current_mpad not in self.known_mpad:
                    print("smth diff")
                    self.known_mpad.append(self.current_mpad)
                    print("appended data!: {}".format(self.known_mpad))
                    self.mpad_pub.publish(self.known_mpad)
                else:
                    special = self.check_special(self.current_mpad)
                    if special is False:
                        self.mpad_pub.publish(self.known_mpad)

                    elif special is True:
                        rescured = self.check_rescured(self.current_mpad)
                        if rescured is False:
                            self.known_mpad.append(self.current_mpad)
                            self.mpad_pub.publish(self.known_mpad)
                        elif rescured is True:
                            self.mpad_pub.publish(self.known_mpad)

                    
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
        
    def send_mpad(self):
        while not rospy.is_shutdown():
            self.mpad_pub.publish(self.known_mpad)


    def start(self):
        response_thread = threading.Thread(target=self.receive_command)
        response_thread.start()

        send_mpad_thread = threading.Thread(target=self.send_mpad)
        send_mpad_thread.start()

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
