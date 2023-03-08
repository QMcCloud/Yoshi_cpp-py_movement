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

import math
import numpy as np

import rclpy
from rclpy import qos
from rclpy.node import Node

from irobot_create_msgs.msg import IrIntensityVector
from geometry_msgs.msg import Twist


class Reactor(Node):

    def __init__(self):
        super().__init__('Reactor')
        self.subscription = self.create_subscription(
            IrIntensityVector,
            "/yoshi/ir_intensity",
            self.ir_callback,
            qos.qos_profile_sensor_data
            )
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, '/yoshi/cmd_vel', 10)

        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.pub_cmd_vel_callback)
        
        self.twist = Twist()

        self.max_turn_rate = 2.0 # 2.0 rad/s
        self.max_linear_rate = 0.1 # 0.2 m/s

    
    def pub_cmd_vel_callback(self):
        self.publisher_.publish(self.twist)
        #self.get_logger().info('Publishing: "%s"' % self.twist)

    def ir_callback(self, msg : IrIntensityVector):
        values = list(map(lambda m: m.value, msg.readings ))

        # turn rate = Left - Right
        # values range +- 3 * 4096 = +- 12288
        turn_rate = 0
        left_intensity = sum(values[:3])
        right_intensity = sum(values[-1:-4:-1])
        if (left_intensity > right_intensity):
            turn_rate = -1.0 * left_intensity
        else:
            turn_rate = right_intensity

        # linear sensitivity scaling 
        # values range +- 12288 / 128
        turn_rate /= 128.0 

        # nonlinear sigmoid normalization
        # values range +- 1
        turn_rate = (2 / (1 + np.exp(-turn_rate))) - 1

        # make turn rate and forward rate froma  2d unit vector
        # values range +- 1
        forward_rate = math.sqrt(1 - turn_rate**2)

        self.twist.angular.z = self.max_turn_rate * turn_rate
        self.twist.linear.x = self.max_linear_rate * forward_rate

        self.get_logger().info('IR: "%s",' % values)
        self.get_logger().info('TWIST: "%s"' % self.twist)
    


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