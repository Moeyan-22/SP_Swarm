#! /usr/bin/env python3

import rospy
import queue
import numpy as np
import json
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Point



class DroneController:

    """
    single Tello Edu Controller written by Moeyan-22

    Contains Utility function for
     - Sending commands to tello driver
     - Creates Queue for commands that is executed every 2secs
     - Listen and respond to takeoff command published in group topic
     - Listen to seqeunce_command and starts only when index has been called
     - Listens to uwb point message from appropriate topic
     - Calculates and sends path finding commands
     - sends "state?" commands alongside any message in queue

    Launch Params:
     - name 
     - id
     - group
     - target (1D array)
    
    """

    def __init__(self):

        rospy.init_node('drone_controller', anonymous=True)
        self.cmd_queue = queue.Queue()

        self.name = rospy.get_param('~name', 'tello')
        self.id = int(rospy.get_param('~id', 0))
        self.group = rospy.get_param('~group', 'A')
        self.target_raw_string = rospy.get_param('~target', '[]')
        self.target_raw = json.loads(self.target_raw_string)

        self.target = []
        self.takeoff = 0
        self.started = False
        self.current_sequence = 0
        self.x = 0
        self.y = 0
        self.total_steps = 20
        self.step = 0.1
        self.step_interval = rospy.Rate(1/0.1) 
        self.base_speed = 0.3
        self.max_pos_error = 0.1
        self.max_movement = 100
        
        self.command_pub = rospy.Publisher('/{}/cmd'.format(self.name), String, queue_size=10)
        self.uwb_sub = rospy.Subscriber('/{}/uwb'.format(self.name), Point, self.get_uwb, queue_size=10)
        self.takeoff_sub = rospy.Subscriber('/{}/takeoff_command'.format(self.group), Int32, self.get_takeoff_command, queue_size=10)
        self.sequence_sub = rospy.Subscriber('/{}/sequence_command'.format(self.group), Int32, self.get_sequence, queue_size=10)


    def get_takeoff_command(self, data):
        self.takeoff = data.data
        if self.takeoff == 1:
            self.execute_takeoff()
            self.takeoff += 1

    def execute_takeoff(self):
        self.cmd_queue.put("command")
        self.cmd_queue.put("mon")
        self.cmd_queue.put("mdirection 0")
        self.cmd_queue.put("takeoff")

    def get_uwb(self, data):
        self.x = data.x
        self.y = data.y

    def process_target_raw(self):
        self.target = self.target_raw
        self.positioning()
        
    def positioning(self):
        if self.started:
            for i in range(len(self.target)):
                self.cmd_queue.put(self.target[i])


    def get_sequence(self, data):
        self.current_sequence = data.data
        self.check_seqeunce()

    def check_seqeunce(self):
        if self.current_sequence == self.id and self.started == False:
            self.started = True
        

    def timing_control(self):

        i = 0

        while not rospy.is_shutdown():
            if not self.cmd_queue.empty():
                cmd = self.cmd_queue.get()
                self.command_pub.publish(cmd)
                i += self.step
                self.step_interval.sleep()
            else:
                pass

            while i < self.total_steps:
                self.command_pub.publish("state?")
                i += self.step
                self.step_interval.sleep()

                if i == self.total_steps:
                    i = 0

    def start(self):
        self.process_target_raw()
        self.timing_control()


if __name__ == '__main__':
    try:
        tello_controller = DroneController()
        tello_controller.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Tello controller: {}".format(e))






                