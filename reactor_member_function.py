import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import String

from irobot_create_msgs.msg import HazardDetectionVector
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class Reactor(Node):

#4 cases: one where both sides of the IR increase so it has to stop and turn, one where the left IR is increasing so it has to turn right, one where the right IR is increasing so it turns left, and one where both are clear so it can just move forward
