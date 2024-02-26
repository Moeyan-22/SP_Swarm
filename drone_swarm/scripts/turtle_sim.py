#! /usr/bin/env python3

import rospy
import threading
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import tkinter as tk
from turtle import RawTurtle, ScrolledCanvas


class TurtleSwarm:

    def __init__(self):

        rospy.init_node('turtle_sim', anonymous=True)
        self.turtles = []
        self.turtle_subscribers = []
        self.threads = []
        self.total_turtle = 1
        self.turtle_starting_position = [
            [0,0]
        ]
        
        self.root = tk.Tk()
        self.root.title('Turtle Controller')
        self.canvas = ScrolledCanvas(self.root)
        self.canvas.pack()
        self.create_turtle_objects()
        self.canvas.bind("<Motion>", self.mouse_motion_callback)
        self.mouse_pos_pub = rospy.Publisher('mouse_pose', Pose, queue_size=10)

    def create_turtle_objects(self):
        for i in range(len(self.total_turtle)):
            turtle = RawTurtle(self.canvas)
            turtle.speed(2)
            turtle.shape("circle")
            turtle.fillcolor("black")
            turtle.penup()
            turtle.goto(self.turtle_starting_position[i][0], self.turtle_starting_position[i][1])
            turtle.pendown()
            self.turtles.append(turtle)
            self.start_turtle(i)

    def start_turtle(self, i):
        (self.turtles[i]).action = None
        turtle_action_subscriber = rospy.Subscriber('/{}/action'.format('tello' + str(i)), String, lambda data: self.get_action(data, i), callback_args=i, queue_size=10)
        self.turtle_subscribers.append(turtle_action_subscriber)
    
    def get_action(self, data, i):
        (self.turtles[i]).action = data.data

    def start_action(self):
        for i in range(len(self.turtle_subscribers)):
            thread = threading.Thread(target=self.execute_action, args=(i,))
            self.threads.append(thread)
            thread.start()

    def execute_action(self, i):
        while not rospy.is_shutdown():

            (self.turtles[i]).x = 0
            (self.turtles[i]).y = 0
            rate = rospy.Rate(20)

            command_received = (self.turtles[i]).action
            if command_received == "command":
                self.turtles[i].fillcolor("red")
            elif command_received == "takeoff":
                self.turtles[i].fillcolor("green")            
            elif "rc" in command_received:
                numbers = [int(s) for s in command_received.split() if s.isdigit()]
                (self.turtles[i]).x = numbers[0]
                (self.turtles[i]).y = numbers[1]

            (self.turtles[i]).forward((self.turtles[i]).x)
            (self.turtles[i]).right((self.turtles[i]).y)

            rate.sleep()

    def mouse_motion_callback(self, event):
        pose_msg = Pose(x=event.x, y=self.canvas.winfo_reqheight() - event.y, theta=0.0)
        self.mouse_pos.publish(pose_msg)

    def start(self):
        self.create_turtle_objects()
        self.start_action()
        self.root.mainloop()




if __name__ == '__main__':
    try:
        pass
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at turtle sim: {}".format(e))