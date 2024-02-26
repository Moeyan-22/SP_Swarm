#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from drone_swarm.msg import Array


import socket
import threading

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

    
    """

    def __init__(self):


        rospy.init_node('drone', anonymous=True)
        
        self.name = rospy.get_param('~name', 'tello')
        self.id = rospy.get_param('~id', 0)
        self.drone_ip = rospy.get_param('~drone_ip', '192.168.0.101')
        self.local_port = rospy.get_param('~local_port', 9010)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', self.local_port))
        self.response = ''
        self.mpad = 0
        self.known_mpad = []
        self.rate = rospy.Rate(20)
        self.rescue = False

        self.action_pub = rospy.Publisher('/{}/action'.format(self.name), String, queue_size=10)
        self.mpad_pub = rospy.Publisher('mpad', Array, queue_size=10)
        self.mpad_sub = rospy.Subscriber('mpad', Array, self.get_mpad, queue_size=10)
        self.command_sub = rospy.Subscriber('/{}/cmd'.format(self.name), String, self.send_command, queue_size=10)

    def send(self, message = ''):
        try:
            self.sock.sendto(message.encode(), (self.drone_ip, 8889))
        except Exception as e:
            rospy.logerr("Error sending message: " + str(e))

    def send_command(self, command = ''):
        if self.rescue == False:
            rospy.loginfo(f"executing command: {command.data}")
            self.send(command.data)
            self.action_pub.publish(command)
        else:
            self.rescue_action()

    def rescue_action(self):
        self.send('stop')
        self.send('go 100 0 0 10 {}'.format(self.mpad))

    def check(self, message):
        if message != 'ok' and message != 'error':
            self.mpad = message.find('mpad:')
            if self.mpad != -1:
                self.mpad = int(message[self.mpad + 5])
 

    def receive_command(self):
        while not rospy.is_shutdown():
            self.response, _ = self.sock.recvfrom(128)
            self.response = self.response.decode(encoding='utf-8').strip()
            rospy.loginfo(f"Received reply from drone: {self.response}")
            self.check(self.response)
            self.rate.sleep()

    def get_mpad(self, data):
        self.known_mpad = data

    def update_mpad(self):
        while not rospy.is_shutdown():
            for mpad in self.known_mpad:
                if self.mpad != -1:
                    self.check_mpad(mpad)
                else:
                    pass

            self.rate.sleep()

    def check_mpad(self, mpad):

        special = False
        rescured = False

        if self.mpad != mpad:
            self.rescue = True
            self.known_mpad.append(self.mpad)
            self.mpad_pub.publish(self.known_mpad)
        elif self.mpad == mpad:
            special = self.check_special(mpad)
            if special == False:
                pass
            elif special == True:
                rescured = self.check_rescured(mpad)
                if rescured == False:
                    self.rescue = True
                    self.known_mpad.append(self.mpad)
                    self.mpad_pub.publish(self.known_mpad)
                elif rescured == True:
                    pass
    

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
        # Start the receive_response thread
        response_thread = threading.Thread(target=self.receive_command())
        response_thread.start()

        self.update_mpad

        # Spin until the node is shut down
        rospy.spin()

if __name__ == '__main__':
    try:
        drone = DroneNode()
        drone.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Tello driver: {}".format(e))
