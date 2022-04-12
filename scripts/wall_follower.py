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

angle_forward = 0
angle_right = 270
angle_off = 1
lidar_too_close = 0.1
forward_speed = 0.05
angular_speed = 0.05
dist_target = 0.55
dist_opt = 0.45

class StopAtWall(object):
    """ This node walks the robot to a wall, and follows it clockwise """

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

    def find_nearest_point(self, ranges):
        # Return the nearest point and the angle to it

        best_dist = 1000
        best_angle = 0

        for angle, dist in enumerate(ranges):
            # No distance data
            if dist == 0:
                continue
            # Point is too close, probably an error
            if dist <= lidar_too_close:
                continue

            if dist < best_dist:
                best_dist = dist
                best_angle = angle

        return best_dist, best_angle

    def angles_close(self, angle0, angle1, offset=20):
        # Returns True if the angles are within offset degrees of each other
        return abs(self.angle_err(angle0, angle1)) < offset

    def angle_err(self, angle0, angle1):
        # Returns the error between the angles
        # The returned value is from -180 to 180, and can be passed to
        # turn_towards() where angle0 is a desired direction and angle1
        # is the current direction
        return ((angle0 - angle1 + 180) % 360) - 180

    def turn_towards(self, angle, speed=angular_speed, p=None):
        # Turns the robot towards the angle

        # Proportional control
        if p is not None:
            speed = min(angular_speed + abs(angle)*p, 0.25)
        self.twist.angular.z = speed * (1 if angle>0 else -1)

        print('    TURN %.1f' % (10*self.twist.angular.z))

    def move_forward(self):
        self.twist.linear.x = forward_speed



    def follow_wall(self, data):
        # Main logic

        self.twist.linear.x = 0

        dist, angle = self.find_nearest_point(data.ranges)

        # If not near a wall
        if dist > dist_target:
            print('  APPROACH')
            # Rotate towards the nearest wall
            self.turn_towards(self.angle_err(angle, angle_forward), p=0.01)
            
            # Approach the wall if it is in front of us
            if self.angles_close(angle, angle_forward):
                self.move_forward()

            return

        # If the wall is not perfectly on the right, rotate to correct
        if dist > dist_opt:
            print('  FAR')
            angle_right_corrected = angle_right + angle_off
        else:
            print('  CLOSE')
            angle_right_corrected = angle_right - angle_off
        self.turn_towards(self.angle_err(angle, angle_right_corrected), p=0.01)

        # If everything so far is good, it's safe to move forward
        if self.angles_close(angle, angle_right):
            self.move_forward()



    def process_scan(self, data):
        self.follow_wall(data)
        self.twist_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = StopAtWall()
    node.run()
