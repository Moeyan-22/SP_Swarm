#! /usr/bin/env python3

import rospy
from std_msgs.msg import String

def hello_publisher():
    # Initialize the ROS node
    rospy.init_node('hello_publisher_node', anonymous=True)

    # Create a publisher for the 'hello_topic' with String type messages
    pub = rospy.Publisher('hello_topic', String, queue_size=10)

    # Set the publishing rate (in Hz)
    rate = rospy.Rate(1)  # 1 Hz

    # Main loop
    while not rospy.is_shutdown():
        # Publish the "hello" message on the 'hello_topic'
        hello_msg = "Hello, ROS!"
        rospy.loginfo(hello_msg)
        pub.publish(hello_msg)

        # Sleep to maintain the specified publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        hello_publisher()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")