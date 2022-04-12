#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist, Point, PointStamped
from std_msgs.msg import Header

class SendVelocities(object):
    def __init__(self):
        rospy.init_node('send_velocities')
        self.point_location_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.forward_speed = 0.2
        self.turn_speed = 0.5
        self.forward_time = 6
        self.turn_time = 3.5

    def run(self):
        # Give publisher time to set up
        time.sleep(1)
        # Loop each leg of the square
        for _ in range(4):
            self.go_forward()
            self.turn()
        # Once the square is over, stop all motion
        self.stop()

    def go_forward(self):
        # Move forward for 6 seconds
        msg = Twist()
        msg.linear.x = self.forward_speed
        msg.angular.z = 0.0

        r = rospy.Rate(2)
        stime = time.time()
        while time.time()-stime < self.forward_time and not rospy.is_shutdown():
            self.point_location_pub.publish(msg)
            r.sleep()

    def turn(self):
        # Turn right for 3.5 seconds
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.turn_speed

        r = rospy.Rate(2)
        stime = time.time()
        while time.time()-stime < self.turn_time and not rospy.is_shutdown():
            self.point_location_pub.publish(msg)
            r.sleep()

    def stop(self):
        # Stop all motion
        # Useful for end of script
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.point_location_pub.publish(msg)
            r.sleep()

if __name__ == '__main__':
    node = SendVelocities()
    node.run()
