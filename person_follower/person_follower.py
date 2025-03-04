# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


import numpy as np
#import ros2_numpy as rnp


class PersonFollower(Node):

    distance_limit = 0.30      #metres
    
    def __init__(self):
        super().__init__('person_follower')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, input_msg):
        angle_min = input_msg.angle_min
        angle_max = input_msg.angle_max
        angle_increment = input_msg.angle_increment
        ranges = np.array(input_msg.ranges)
        #
        # your code for computing vx, wz
        #
        laser_scan_angle_vec = np.arange(angle_min, angle_max, angle_increment)
        
        captured_values = np.logical_not(np.logical_or(
            np.logical_or(np.isnan(ranges), np.isinf(ranges)),
            np.logical_or((ranges < input_msg.range_min), (ranges > input_msg.range_max))
        ))
        
        #num_points = np.sum(captured_values)
        
        #if num_points > 0:
        if np.sum(captured_values) > 0:
            mean_position = np.sum(ranges[captured_values] * ranges[captured_values]) / np.sum(ranges[captured_values])
            mean_angle = np.sum(laser_scan_angle_vec[captured_values] * ranges[captured_values]) / np.sum(ranges[captured_values])
            mean_direction_vec = np.array([np.cos(mean_angle), np.sin(mean_angle), 0], dtype=float)
            mean_angle_vec = np.array([0, 0, mean_angle])
            
            # turtlebot speed
            mean_velocity = (mean_position - self.distance_limit)   
            vx = mean_velocity * mean_direction_vec
            vx[vx > 2] = 2
            vx[vx < -2] = -2
            wz = mean_angle_vec * 5
            wz[wz > np.pi/8] = np.pi/8
            wz[wz < -np.pi/8] = -np.pi/8
            
        else:
            vx = np.array([0,0,0], dtype=float)
            wz = np.array([0,0,0], dtype=float)
        #
        #vx = 0.
        #wz = 0.
        #
        output_msg = Twist()
        output_msg.linear.x = vx[0].astype(float)
        output_msg.linear.y = vx[1].astype(float)
        output_msg.linear.z = vx[2].astype(float)
        output_msg.angular.x = wz[0].astype(float)
        output_msg.angular.y = wz[1].astype(float)
        output_msg.angular.z = wz[2].astype(float)
        self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    person_follower = PersonFollower()
    rclpy.spin(person_follower)
    person_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
