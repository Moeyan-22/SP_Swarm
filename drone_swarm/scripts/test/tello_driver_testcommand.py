#! /usr/bin/env python3

import rospy
from std_msgs.msg import String

class command:
    def __init__(self):
        rospy.init_node('drone', anonymous=True)


    def sendcommand(self):
        rate = rospy.Rate(0.1) # 10hz
        while not rospy.is_shutdown():
            pub = rospy.Publisher('/tello2/cmd', String, queue_size=10)
            pub.publish("command")
            rate.sleep()

if __name__ == '__main__':
    try:
        drone = command()
        drone.sendcommand()
    except rospy.ROSInterruptException:
        pass
