#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.scan_sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # TODO: set PID gains
        self.kp = 1.0
        self.kd = 2.0
        self.ki = 0.0

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        # TODO: store any necessary values you think you'll need
        self.desired_distance = 1.0  
        self.lookahead_distance = 1.0
        self.angle_ahead = np.radians(45.0)

    def get_range(self, range_data, angle, angle_min, angle_inc):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement

        index = int(round((angle - angle_min) / angle_inc))
        index = max(0, min(index, len(range_data) - 1))  # makes sure the index is within bounds
        r = range_data[index]
        if np.isnan(r) or np.isinf(r):
            return 10.0
        return r

    def get_error(self, msg, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        b= self.get_range(msg.ranges, np.pi / 2, msg.angle_min, msg.angle_increment)
        a= self.get_range(msg.ranges, np.pi / 2 - self.angle_ahead, msg.angle_min, msg.angle_increment)
        alpha = np.arctan((a*np.cos(self.angle_ahead) - b) / (a * np.sin(self.angle_ahead)))
        Dt = b * np.cos(alpha)
        Dt1 = Dt + self.lookahead_distance * np.sin(alpha)
        error = Dt1 - dist
        
        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """

        # TODO: Use kp, ki & kd to implement a PID controller
        p = self.kp * error
        d = self.kd * (error - self.prev_error)
        self.integral += error
        i = self.ki *self.integral
        angle = p + d + i
        self.prev_error = error

        # TODO: fill in drive message and publish
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        self.drive_pub.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        error = self.get_error(msg, self.desired_distance) # TODO: replace with error calculated by get_error()
        velocity = 1.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()