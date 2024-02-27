#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import time

class TakeoffCommandPublisher:

    def __init__(self):
        rospy.init_node('takeoff_command_publisher', anonymous=True)
        self.takeoff_command_pub = rospy.Publisher('/B/takeoff_command', Int32, queue_size=10)
        self.rate = rospy.Rate(1)  # Adjust the publishing rate as needed

    def publish_takeoff_command(self):
        takeoff_command = Int32()
        takeoff_command.data = 1  # You can set the value according to your needs
        self.takeoff_command_pub.publish(takeoff_command)

    def start(self):
        time.sleep(2)
        print("started takeoff")
        while not rospy.is_shutdown():
            self.publish_takeoff_command()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        takeoff_publisher = TakeoffCommandPublisher()
        takeoff_publisher.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    except Exception as e:
        rospy.logerr("Error in TakeoffCommandPublisher: {}".format(e))
