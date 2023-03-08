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

import rclpy
from rclpy import qos
from rclpy.node import Node

from std_msgs.msg import String

from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class Wanderer(Node):

    def __init__(self):
        super().__init__('wanderer')
        self.subscription = self.create_subscription(
            HazardDetectionVector,
            "/yoshi/hazard_detection",
            self.hazard_callback,
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
        self.twist.linear.x = 0.1
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0

        self.do_once_timer = None

        self.prev_hazard = []

    
    def pub_cmd_vel_callback(self):
        self.publisher_.publish(self.twist)
        self.get_logger().info('Publishing: "%s"' % self.twist)
    
    def do_once(self, func=None):
        self.do_once_timer.destroy()
        self.get_logger().info('DOONCE FUNC!: "%s"' % func)
        if(func) : func()
        self.get_logger().info('IS DOONCE TIMER DEAD!: "%s"' % self.do_once_timer)

    def hazard_callback(self, msg : HazardDetectionVector):
        # if(len(msg.detections) > 0):
        #     self.get_logger().info('hazardLen!: "%i"' % len(msg.detections))
        actions = {
            HazardDetection.BUMP: ("BUMP",self.handle_bump),
            HazardDetection.BACKUP_LIMIT : ("BACKUP_LIMIT",self.handle_backup_limit),
            HazardDetection.CLIFF: ("CLIFF",self.handle_bump),
            HazardDetection.WHEEL_DROP : ("WHEEL_DROP",self.stop),
            HazardDetection.STALL : ("STALL",self.handle_bump),
            HazardDetection.OBJECT_PROXIMITY : ("OBJECT_PROXIMITY",self.handle_bump)
        }
        newHazardList = []
        for hazard in msg.detections:
            newHazardList.append(hazard.type)
            if (hazard.type not in self.prev_hazard):
                hazardName, action = actions.get(hazard.type, None)
                self.get_logger().info('hazardType!: "%s"' % hazardName)
                self.get_logger().info('RUNNING ACTION!: "%s"' % action)
                if (action):
                    action()
        self.prev_hazard = newHazardList
        
            
    
    def stop(self):
        self.get_logger().info('STOP!')
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.publisher_.publish(self.twist)
    
    def reverse(self):
        self.get_logger().info('REVERSE!')
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.twist.linear.x = -0.01
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.publisher_.publish(self.twist)
    
    def forward(self):
        self.get_logger().info('FORWARD!')
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.twist.linear.x = 0.1
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.publisher_.publish(self.twist)
    
    def turn(self):
        self.get_logger().info('TURN!')
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = choice([-1.0, 1.0])
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.publisher_.publish(self.twist)

    def handle_bump(self):
        self.stop()
        if self.do_once_timer : self.do_once_timer.destroy()
        self.do_once_timer = self.create_timer(1.0, lambda : self.do_once(func = self.handle_backup_limit))
        self.reverse()
    
    def handle_backup_limit(self):
        self.stop()
        if self.do_once_timer : self.do_once_timer.destroy()
        delay = uniform(2.0943951, 4.1887902)
        self.get_logger().info('WAIT! : %f' % delay)
        self.do_once_timer = self.create_timer(delay, lambda : self.do_once(func = self.forward))
        self.turn()


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
