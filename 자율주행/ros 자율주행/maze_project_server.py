#!/usr/bin/env/ python3

# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# https://docs.ros.org/en/foxy/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html#id4

import math
import time

from custom_interfaces.action import Maze

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from rclpy.executors import MultiThreadedExecutor


# Maze.action structure
#     int32[] turning_sequence
#     ---
#     bool success
#     ---
#     string feedback_msg


direction_dict = {0: (-1 * math.pi / 2), 1: math.pi, 2: math.pi / 2, 3: 0.0}
direction_str_dict = {0: 'Up', 1: 'Right', 2: 'Down', 3: 'Left'}


class MazeActionServer(Node):

    def __init__(self):
        super().__init__('maze_server')
        self.yaw = 0.0
        self.forward_distance = 0.0

        self.twist_msg =  Twist()
        #self.loop_rate = self.create_rate(5, self.get_clock())

        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_sub_cb, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        #timer_period = 0.1

        self.action_server = ActionServer(
            self,
            Maze,   #action type
            'maze', #action 이름
            self.execute_callback,  #action 목표를 받으면 실행되는 콜백함수
            #goal_callback=self.goal_callback,       # goal response가 오면 우선 goal_callback 을 실행시킨뒤 execute_callback을 실행
        )

        self.get_logger().info('=== Fibonacci Action Server Started ====')

    def laser_sub_cb(self,data):
        self.forward_distance = data.ranges[360]

    def turn_robot(self, euler_angle):
        self.get_logger().info(f'Robot turn to {euler_angle}')

        turn_offset = 100
        while abs(turn_offset) > 0.087:
            turn_offset = 0.7 * (euler_angle - self.yaw)
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = turn_offset
            self.cmd_vel_pub.publish(self.twist_msg)

        self.stop_robot()

    def stop_robot(self):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_msg)

        time.sleep(1)

    def park_robot(self):
        while self.forward_distance > 1.0:
            self.twist_msg.linear.x = 0.5
            self.twist_msg.angular.z = 0.0

            self.cmd_vel_pub.publish(self.twist_msg)

        self.stop_robot()

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Maze.Feedback()
        feedback_msg.feedback_msg = ""

        for _, val in enumerate(goal_handle.request.truning_sequence):
            self.get_logger().info(f'current cmd : {val}')

            feedback_msg.feedback_msg = f'Turning {direction_str_dict[val]}'

            #self.turn_robot()
            #self.park_robot()

            goal_handle.publish_feedback(feedback_msg)

        return Maze.Result()

    # def goal_callback(self, goal_request):
    #     """Accept or reject a client request to begin an action."""
    #     # This server allows multiple goals in parallel
    #     self.get_logger().info('Received goal request')
    #     return GoalResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)

    try:
        maze_action_server = MazeActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(maze_action_server)
        try:
            executor.spin()
        except KeyboardInterrupt:
            maze_action_server.destory()
            maze_action_server.destriy_node()
        finally:
            rclpy.shutdown()



if __name__ == '__main__':
    main()