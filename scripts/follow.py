#!/usr/bin/env python3

# TOPICS:
#   cmd_vel: publish to, used for setting robot velocity
#   scan   : subscribing, where the wall is

import rospy

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# How close we will get to wall.
distance = 0.4
speed = 0.75

class StopAtWall(object):
    """ This node makes the robot follow the closest object """

    def __init__(self):
        # Start rospy node.
        rospy.init_node("walk_to_wall")

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Get a publisher to the cmd_vel topic.
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

    def process_scan(self, data):
        # Find the closest point and the angle to it
        closest_idx = 0
        closest_dist = 10
        for i, d in enumerate(data.ranges):
            if d != 0 and d < closest_dist:
                closest_idx = i
                closest_dist = d

        print('%3d  %0.4f   ' % (closest_idx, closest_dist), end='')
        self.twist.linear.x = 0
        if closest_dist > 1.5 or closest_dist < 0.3:
            # Points that are too close are error/noise, and too far away we filter out
            self.twist.angular.z = 0
            print('Bad dist')
        elif closest_idx < 20 or closest_idx > 340:
            # If we are mostly facing the person, move towards them
            self.twist.angular.z = 0
            print('Centered')
            if closest_dist > 0.5:
                self.twist.linear.x = 0.5
        elif closest_idx < 180:
            # Rotate left
            self.twist.angular.z = speed
            print('Left')
        else:
            # Rotate right
            self.twist.angular.z = -speed
            print('Right')

        # Publish msg to cmd_vel.
        self.twist_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = StopAtWall()
    node.run()
