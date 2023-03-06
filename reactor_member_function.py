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

from random import choice, uniform
import sys
import math
import numpy as np

import rclpy
from rclpy import qos
from rclpy.node import Node

from std_msgs.msg import String

from irobot_create_msgs.msg import IrIntensity, IrIntensityVector
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class Wanderer(Node):

    def __init__(self):
        super().__init__('wanderer')
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
        self.twist.angular = Vector3()
        self.twist.linear = Vector3()

        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0

    
    def pub_cmd_vel_callback(self):
        self.publisher_.publish(self.twist)
        #self.get_logger().info('Publishing: "%s"' % self.twist)
    
    def remap(self, x : IrIntensity):
        return (2 / (1+ np.exp(-x.value / 512.0))) - 1

    def ir_callback(self, msg : IrIntensityVector):
        values = list(map(self.remap, msg.readings ))

        turnrate = (-1.0 * sum(values[:3])) + sum(values[-1:-4:-1])
        forward_rate = 0.0
        if(turnrate**2 < 9 ):
            forward_rate = 0.025 * math.sqrt(9 - turnrate**2) 
        

        self.get_logger().info('IR: "%s",' % values)
        self.get_logger().info('TR: "%s"' % turnrate)
        self.get_logger().info('FR: "%s"' % forward_rate)

        self.twist.angular.z = 2 * turnrate
        self.twist.linear.x = forward_rate

        pass 
    


def main(args=None):
    rclpy.init(args=args)

    wanderer = Wanderer()

    rclpy.spin(wanderer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wanderer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()