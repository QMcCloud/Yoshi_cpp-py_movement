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

from std_msgs.msg import String

from irobot_create_msgs.msg import HazardDetectionVector
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector


class Reactor(Node):

    def __init__(self):
        super().__init__('reactor')

        self.subscription = self.create_subscription(
            HazardDetectionVector,
            "/yoshi/hazard_detection",
            self.hazard_callback,
            10)

        self.publisher_ = self.create_publisher(Twist, '/yoshi/cmd_vel', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        self.subscription  # prevent unused variable warning

    def hazard_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def stop_callback(self):
        msg = Twist()
        msg.angular = Vector3()
        msg.linear = Vector3()

        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    reactor = Reactor()

    rclpy.spin(reactor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reactor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
