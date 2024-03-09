#! /usr/bin/env python3

import rospy
import queue
import numpy as np
import json
import math
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from drone_swarm.msg import Array
from geometry_msgs.msg import PoseStamped
import time


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
        self.land_x = 0
        self.land_y = 0
        self.i = 0
        self.land_data = []
        self.exit_positioning = 0


        self.total_steps = 2
        self.step = 1
        self.step_interval = rospy.Rate(1/0.1) 
        self.base_speed = 0.3


        
        self.command_pub = rospy.Publisher('/{}/cmd'.format(self.name), String, queue_size=10)

        #real uwb
        #self.uwb_sub = rospy.Subscriber('/nlt_anchorframe0_pose_node{}'.format(self.id), PoseStamped, self.get_uwb, queue_size=10)

        #simulation uwb
        self.uwb_sub = rospy.Subscriber('/{}/uwb'.format(self.name), Point, self.get_fake_uwb, queue_size=10)


        self.takeoff_sub = rospy.Subscriber('/{}/takeoff_command'.format(self.group), Int32, self.get_takeoff_command, queue_size=10)
        self.sequence_sub = rospy.Subscriber('/{}/sequence_command'.format(self.group), Array, self.get_sequence, queue_size=10)
        self.land_sub = rospy.Subscriber('/{}/land_data'.format(self.name), Array, self.get_land_data, queue_size=10)


    def get_takeoff_command(self, data):
        if self.takeoff == 0:
            self.takeoff = data.data
        elif self.takeoff == 1:
            self.execute_takeoff()
            self.takeoff += 1

    def execute_takeoff(self):
        self.cmd_queue.put("command")
        self.cmd_queue.put("mon")
        self.cmd_queue.put("speed 100")
        self.cmd_queue.put("takeoff")



    def execute_landing(self):
        self.cmd_queue = queue.Queue()
        self.target = []
        self.target = [f'{self.land_x} {self.land_y}']
        self.exit_positioning = 1
        print(f"hello my nigga, this is target now {self.target}")



    def get_uwb(self, data):
        x_value = data.pose.position.x
        y_value = data.pose.position.y

        self.x_value = round(x_value * 100, 4)
        self.y_value = round(y_value * 100, 4)

    def get_fake_uwb(self, data):
        x_value = data.x
        y_value = data.y

        self.x_value = x_value
        self.y_value = y_value

    def get_land_data(self, data):
        if self.i < 1:
            self.land_data = data.data
            self.land_x = self.land_data[0]
            self.land_y = self.land_data[1]
            self.execute_landing()
            self.i += 1


    def process_target_raw(self):
        self.target = self.target_raw
        #print(self.target)
        #if self.id != 0 and self.id != 1:
        #    pass
        #for i in range(len(self.target)):
        #    self.cmd_queue.put(self.target[i])

        
    def positioning(self):
        rate = rospy.Rate(5)

        print(self.target)
        for target in self.target:

            if self.exit_positioning == 1:
                target = f"{self.land_x} {self.land_y}"
            
            if self.exit_positioning == 2:
                break
            


            while not rospy.is_shutdown():
                point = Point()
                point.x = 0
                point.y = 0
                alpha = 0.2 #0.5
                rc_command = ""
                numbers = [int(s) for s in target.split() if s.lstrip('-').isdigit()]                
                target_point = np.array([numbers[0], numbers[1]])
                current_point = np.array([self.x_value, self.y_value])

                error_x = target_point[0] - current_point[0]
                error_y = target_point[1] - current_point[1]

                #print(f"error x: {error_x} error y: {error_y}")

                max_output = 100  #70
                control_x = np.clip(error_x, -max_output, max_output)
                control_y = np.clip(error_y, -max_output, max_output)

                control_x = alpha * control_x + (1 - alpha) * point.x
                control_y = alpha * control_y + (1 - alpha) * point.y

                dist = np.linalg.norm(target_point - current_point)

                #print(f"current x:{self.x_value}, current y:{self.y_value}, target:{target} error x:{error_x} error y:{error_y} dist: {dist}")

                min_dist = 10 #20

                if dist < min_dist:
                    break

                if dist < min_dist and self.exit_positioning == 1:
                    self.exit_positioning +=1
                    break

                if dist < min_dist and self.exit_positioning == 2:
                    self.exit_positioning += 3
                    break


                point.x = control_x
                point.y = control_y

                if not math.isnan(point.x) and not math.isnan(point.y):
                    rc_command = "rc {} {} 0 0".format(int(point.x), int(point.y))
                else:
                    rc_command = "rc 0 0 0 0"

                self.cmd_queue.put(rc_command)

                rate.sleep()

        self.cmd_queue.put("land")
        time.sleep(2)
        rospy.signal_shutdown('Self-termination requested')


    def get_sequence(self, data):
        self.current_sequence = data.data
        self.check_seqeunce()

    def check_seqeunce(self):
        if self.id in self.current_sequence and self.started == False:
            self.started = True
            print("drone {} called".format(self.id))                
            self.process_target_raw()
            self.positioning()


    def timing_control(self):

        i = 0
        
        while not rospy.is_shutdown():
            if not self.cmd_queue.empty() and i == 0:
                cmd = self.cmd_queue.get()
                self.command_pub.publish(cmd)
                i += self.step
                self.step_interval.sleep()
            else:
                self.command_pub.publish("rc 0 0 0 0")
                i += self.step
                self.step_interval.sleep()

            while i < self.total_steps and i != 0:
                self.command_pub.publish("mid?")
                i += self.step
                self.step_interval.sleep()

                if i == self.total_steps:
                    i = 0

    def start(self):
        self.timing_control()



if __name__ == '__main__':
    try:
        tello_controller = DroneController()
        tello_controller.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at hardcode_tello_controller {}".format(e))






                