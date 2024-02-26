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
        self.id = rospy.get_param('~id', 0)
        self.group = rospy.get_param('~group', 'A')
        self.target_raw_string = rospy.get_param('~target', '[]')
        self.target_raw = json.loads(self.target_raw_string)
        self.ns = self.name + str(self.id)

        self.target = []
        self.takeoff = 0
        self.started = False
        self.current_sequence = 0
        self.x = 0
        self.y = 0
        self.total_steps = 10
        self.step = 1
        self.step_interval = rospy.Rate(1/0.1) 
        self.base_speed = 0.3
        self.max_pos_error = 0.1
        self.max_movement = 100
        
        self.command_pub = rospy.Publisher('/{}/cmd'.format(self.ns), String, queue_size=10)
        self.uwb_sub = rospy.Subscriber('/{}/uwb'.format(self.ns), Point, self.get_uwb, queue_size=10)
        self.takeoff_sub = rospy.Subscriber('/{}/takeoff_command'.format(self.group), Int32, self.get_takeoff_command, queue_size=10)
        self.sequence_sub = rospy.Subscriber('/{}/sequence_command'.format(self.group), Int32, self.get_sequence, queue_size=10)


    def get_takeoff_command(self, data):
        if self.takeoff == 0:
            self.takeoff = data.data
        elif self.takeoff == 1:
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
        i = 0
        cycles = self.target_raw[0]
        self.target = [[0, 0] for _ in range(cycles)]
        while i < cycles:
            self.target[i][0] = self.target_raw[(i + 1)*2-1]
            self.target[i][1] = self.target_raw[(i + 1)*2]
            i += 1
        self.positioning()
        
    def positioning(self):
        if self.started:
            for target in self.target:

                point = Point()
                point.x = 0
                point.y = 0
                rc_command = ""

                point1 = np.array(target[0])
                point2 = np.array(target[1])

                diff = point2 - point1
                dist = np.linalg.norm(diff)
                speed_multiplier = self.base_speed

                if dist < self.max_pos_error:
                    speed_multiplier *= 0.3

                [point.x, point.y] = speed_multiplier * diff / dist
                [point.x, point.y] = [point.x, point.y] * self.max_movement
                rc_command = "rc {} {} 0 0".format(point.x, point.y)
                self.cmd_queue.put(rc_command)


    def get_sequence(self, data):
        self.current_sequence = data.data
        self.check_seqeunce()

    def check_seqeunce(self):
        if self.current_sequence == self.id and self.started == False:
            self.started = True
        

    def timing_control(self):

        i = 0

        while not rospy.is_shutdown():
            if not self.cmd_queue.empty() and i == 0:
                cmd = self.cmd_queue.get()
                self.command_pub.publish(cmd)
                i += self.step
                self.step_interval.sleep()
            else:
                self.command_pub.publish("state?")
                i += self.step
                self.step_interval.sleep()

            while i < self.total_steps and i != 0:
                self.command_pub.publish("state?")
                i += self.step
                self.step_interval.sleep()

                if i == self.total_steps:
                    i = 0
                    

    def start(self):
        self.timing_control()
        self.process_target_raw()


if __name__ == '__main__':
    try:
        tello_controller = DroneController()
        tello_controller.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at Tello controller: {}".format(e))






                





