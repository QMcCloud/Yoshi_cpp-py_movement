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
            10)
        self.publisher_ = self.create_publisher(Twist, '/yoshi/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.pub_cmd_vel_callback)
        
        self.twist = Twist()
        self.twist.angular = Vector3()
        self.twist.linear = Vector3()

        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0

        self.subscription  # prevent unused variable warning
    
    def pub_cmd_vel_callback(self):
        self.publisher_.publish(self.twist)
        self.get_logger().info('Publishing: "%s"' % self.twist)
    
    def do_once(self, func):
        if(func) : func()
        self.do_once_timer.destroy()

    def hazard_callback(self, msg : HazardDetectionVector):
        actions = {
            HazardDetection.BUMP: self.handle_bump,
            HazardDetection.BACKUP_LIMIT : self.handle_backup_limit,
            HazardDetection.CLIFF: self.handle_bump,
            HazardDetection.WHEEL_DROP : self.stop,
            HazardDetection.STALL : self.handle_bump,
            HazardDetection.OBJECT_PROXIMITY : self.handle_bump
        }
        for hazard in msg.detections:
            action = actions.get(hazard, None)
            if (action):
                action()
            self.get_logger().info('I heard: "%s"' % hazard)
    
    def stop(self):
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.publisher_.publish(self.twist)
    
    def reverse(self):
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.twist.linear.x = -0.01
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.publisher_.publish(self.twist)
    
    def forward(self):
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.twist.linear.x = 0.1
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.publisher_.publish(self.twist)
    
    def turn(self):
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.1
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.publisher_.publish(self.twist)

    def handle_bump(self):
        self.stop()
        if self.do_once_timer : self.do_once_timer.destroy()
        self.do_once_timer = self.create_timer(1, lambda : self.do_once(self.handle_backup_limit))
        self.reverse()
    
    def handle_backup_limit(self):
        self.stop()
        if self.do_once_timer : self.do_once_timer.destroy()
        self.do_once_timer = self.create_timer(1, lambda : self.do_once(self.forward))
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
