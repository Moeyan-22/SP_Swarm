#! /usr/bin/env python3

import rospy
import threading
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import tkinter as tk
from turtle import RawTurtle, ScrolledCanvas
from geometry_msgs.msg import Pose, Point, Quaternion




class TurtleSim:

    def __init__(self):

        rospy.init_node('turtle_sim', anonymous=True)
        self.turtles = []
        self.turtle_subscribers = []
        self.threads = []
        self.total_turtle = 3 #change this to how many drone
        self.turtle_starting_position = [ #change for the position of the drone
            [-100,0],
            [0,0],
            [100,0]
        ]
        
        self.root = tk.Tk()
        self.root.title('Turtle Controller')
        self.canvas = ScrolledCanvas(self.root)
        self.canvas.pack()
        self.canvas.bind("<Motion>", self.mouse_motion_callback)
        self.mouse_pos_pub = rospy.Publisher('mouse_pose', Pose, queue_size=10)

    def create_turtle_objects(self):
        for i in range(self.total_turtle):
            turtle = RawTurtle(self.canvas)
            turtle.speed(2)
            turtle.shape("circle")
            turtle.fillcolor("black")
            turtle.penup()
            turtle.goto(self.turtle_starting_position[i][0], self.turtle_starting_position[i][1])
            turtle.pendown()
            turtle.action = ''
            self.turtles.append(turtle)
            self.start_turtle(i)

    def start_turtle(self, i):
        turtle_action_subscriber = rospy.Subscriber('/{}/action'.format('tello' + str(i)), String, lambda data, i: self.get_action(data, i), callback_args=i, queue_size=10)
        self.turtle_subscribers.append(turtle_action_subscriber)
    
    def get_action(self, data, i):
        (self.turtles[i]).action = data.data

    def start_action(self):
        for i in range(len(self.turtle_subscribers)):
            thread = threading.Thread(target=self.execute_action, args=(i,))
            self.threads.append(thread)
            thread.start()

    def execute_action(self, i):
        print(self.turtles[i].xcor())
        while not rospy.is_shutdown():
            rate = rospy.Rate(20)

            command_received = (self.turtles[i]).action
            if command_received == "command":
                self.turtles[i].fillcolor("red")
            elif command_received == "takeoff":
                self.turtles[i].fillcolor("green")
            elif "rc" in command_received:
                numbers = [int(s) for s in command_received.split() if s.isdigit()]
                x_offset = numbers[0]
                y_offset = numbers[1]
                # Move forward and right independently
                self.turtles[i].goto((self.turtles[i]).xcor() + y_offset, (self.turtles[i]).ycor() + x_offset)  # Move right by 20 pixels
            rate.sleep()

    def mouse_motion_callback(self, event):
        pose_msg = Pose(position=Point(x=event.x, y=self.canvas.winfo_reqheight() - event.y, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        self.mouse_pos_pub.publish(pose_msg)

    def start(self):
        self.create_turtle_objects()
        self.start_action()
        self.root.mainloop()

if __name__ == '__main__':
    try:
        turtle_sim = TurtleSim()
        turtle_sim.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error at turtle sim: {}".format(e))