# Copyright 2021 Seoul Business Agency Inc.
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

# Referenced from Below Link
# https://docs.ros2.org/foxy/api/rclpy/api/init_shutdown.html#rclpy.create_node

# !/usr/bin/env/ python3
import sys
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class Park(Node):
    def __init__(self):
        super().__init__('park_node')
        self.publisher = self.create_publisher( Twist, 'skidbot/cmd_vel', 10)
        self.subscriber = self.create_subscription( LaserScan, 'skidbot/scan', self.sub_callback, 10)
        
        self.publisher
        self.subscriber

    def sub_callback(self, msg):
        twist_msg = Twist()
        distance_forward = msg.ranges[360]

        if distance_forward > 1:
            self.get_logger().info(f'Distance from front object : {distance_forward}')
            twist_msg.linear.x = 0.5
            self.publisher.publish(twist_msg)
    
        else:
            self.get_logger().info(f'==============parking Done!!!==========')
            twist_msg.linear.x = 0.0
            self.publisher.publish(twist_msg)
       




def main(args = None):
    rclpy.init(args=args)

    parking_node = Park()

    try:
        rclpy.spin(parking_node)
    except KeyboardInterrupt:
        print("==============server stopped cleanly ==========")
    except BaseException:
        print("!! Exception in server:", file=sys.stderr)
        raise
    finally:
        rclpy.shutdown()
       

if __name__ == '__main__':
    main()
