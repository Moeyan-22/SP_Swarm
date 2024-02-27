#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class Test:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('hello_publisher_node', anonymous=True)

        # Create a publisher for the 'hello_topic' with String type messages
        self.pub = rospy.Publisher('hello_topic', String, queue_size=10)

        # Set the publishing rate (in Hz)
        self.rate = rospy.Rate(1)  # 1 Hz
        self.published = 0

    def start(self):

        # Main loop
        while not rospy.is_shutdown():
            # Publish the "hello" message on the 'hello_topic'
            if self.published < 5:
                hello_msg = "I love ROS!"
                rospy.loginfo(hello_msg)
                self.pub.publish(hello_msg)
                self.published += 1

            # Sleep to maintain the specified publishing rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        test_node = Test()
        test_node.start()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
