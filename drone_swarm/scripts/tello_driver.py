#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from drone_swarm.msg import Array


import socket
import threading

class DroneNode:

    """
    
    Single Tello Edu Controller written by Moe

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

        self.action_pub = rospy.Publisher('/{}/action'.format(self.ns), String, queue_size=10)
        self.mpad_pub = rospy.Publisher('mpad', Array, queue_size=10)
        self.mpad_sub = rospy.Subscriber('mpad', Array, self.get_mpad, queue_size=10)
        self.command_sub = rospy.Subscriber('/{}/cmd'.format(self.ns), String, self.send_command, queue_size=10)

    def send(self, message = ''):
        try:
            self.sock.sendto(message.encode(), (self.drone_ip, 8889))
        except Exception as e:
            rospy.logerr("Error sending message: " + str(e))

    def send_command(self, command = ''):
        rospy.loginfo(f"Received command: {command.data}")
        self.send(command.data)
        self.action_pub.publish(command)

    def check(self, message):
        if message != 'ok' or message != 'error':
            self.mpad = int(message)
            self.update_mpad

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
        for mpad in self.known_mpad:
            if self.mpad != -1:
                if self.mpad != mpad:
                    self.known_mpad.append(self.mpad)
            self.mpad_pub.publish(self.known_mpad)


    def start(self):
        # Start the receive_response thread
        response_thread = threading.Thread(target=self.receive_response)
        response_thread.start()

        # Spin until the node is shut down
        rospy.spin()

if __name__ == '__main__':
    try:
        drone = DroneNode()
        drone.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
