#! /usr/bin/env python3

import rospy
import threading
from geometry_msgs.msg import Point
from std_msgs.msg import String
import tkinter as tk
from turtle import RawTurtle, ScrolledCanvas
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseStamped
import time




class TurtleSim:

    def __init__(self):

        rospy.init_node('turtle_sim', anonymous=True)

        self.total_turtle = rospy.get_param('~total_turtle', 1)
        self.turtle_starting_position = [[0, 0] for _ in range(self.total_turtle)]
        self.edit_turtle_starting_position()
        self.turtles = []
        self.turtle_subscribers = []
        self.turtle_publishers = []

        
        self.root = tk.Tk()
        self.root.title('Turtle Controller')
        self.canvas = ScrolledCanvas(self.root, width=1000, height=1000) #1:2 scale
        self.canvas.pack()
        self.canvas.bind("<Motion>", self.mouse_drag_callback)
        self.mouse_pos_pub = rospy.Publisher('mouse_pose', Pose, queue_size=10)
        self.left_button_held = False

        self.uwb_sub = rospy.Subscriber('/nlt_anchorframe0_pose_node0', PoseStamped, self.get_uwb_coords, queue_size=10)


    def get_uwb_coords(self, data):

        scale_factor = 1
        x_offset = 150
        y_offset = 300
        draw_uwb = True

        x_value = data.pose.position.x
        y_value = data.pose.position.y

        x_value = round(x_value * 100 * scale_factor - x_offset, 4)
        y_value = -round(y_value * 100 * scale_factor - y_offset, 4)

        print(f"x value {x_value}, y value {y_value}")



        if draw_uwb == True:
            radius = 5
            self.canvas.create_oval(
                x_value - radius, y_value - radius,
                x_value + radius, y_value + radius,
                fill="blue"
            )

            self.root.update()



    def edit_turtle_starting_position(self):
        for i in range(self.total_turtle):
                self.turtle_starting_position[i][1] = -200 - (i-1)*50
                self.turtle_starting_position[i][0] = -100



    def create_turtle_objects(self):
        for i in range(self.total_turtle):
            turtle = RawTurtle(self.canvas)
            turtle.speed(1)
            turtle.shape("circle")
            turtle.fillcolor("black")
            turtle.penup()
            turtle.goto(self.turtle_starting_position[i][0], self.turtle_starting_position[i][1])
            turtle.pendown()
            turtle.action = ''
            self.turtles.append(turtle)
            self.start_turtle(i)
            self.start_coords(i)

    def start_turtle(self, i):
        turtle_action_subscriber = rospy.Subscriber('/{}/cmd'.format('tello' + str(i)), String, self.get_action, callback_args=(i), queue_size=10)
        self.turtle_subscribers.append(turtle_action_subscriber)

    def get_action(self, data, i):
        self.turtles[i].action = data.data        

    def start_coords(self, i):
        turtle_action_publisher = rospy.Publisher('/{}/uwb'.format('tello' + str(i)), Point, queue_size=10)
        self.turtle_publishers.append(turtle_action_publisher)        
        thread_pose = threading.Thread(target=self.get_coords, args=(i,))
        thread_pose.start()

    def start_action(self):
        for i in range(len(self.turtle_subscribers)):
            thread_action = threading.Thread(target=self.execute_action, args=(i,))
            thread_action.start()


    def get_coords(self, i):
        while not rospy.is_shutdown():

            rate = rospy.Rate(40)

            pose_msg = Point()

            pose_msg.x = self.turtles[i].xcor()
            pose_msg.y = self.turtles[i].ycor()

            self.turtle_publishers[i].publish(pose_msg)

            rate.sleep()

        

    def execute_action(self, i):
        while not rospy.is_shutdown():

            rate = rospy.Rate(20)

            command_received = self.turtles[i].action
            if command_received == "command":
                self.turtles[i].fillcolor("red")
            elif command_received == "takeoff":

                if i == 0:
                    self.turtles[i].fillcolor("green")
                elif i == 1:
                    self.turtles[i].fillcolor("yellow")
                elif i == 2:
                    self.turtles[i].fillcolor("purple")


            elif command_received == "land":
                self.turtles[i].fillcolor("blue")
            elif "rc" in command_received:
                numbers = [int(s) for s in command_received.split() if s.lstrip('-').isdigit()]                
                y_offset = numbers[0]
                x_offset = numbers[1]
                
                self.turtles[i].goto(self.turtles[i].xcor() + y_offset, self.turtles[i].ycor() + x_offset)  
            rate.sleep()

    def mouse_drag_callback(self, event):
        adjusted_x = event.x - 505
        adjusted_y = -event.y + 445
        pose_msg = Pose(position=Point(x=adjusted_x, y=adjusted_y, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
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