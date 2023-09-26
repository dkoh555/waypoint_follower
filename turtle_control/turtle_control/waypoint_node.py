"""
???Publishes twist that will move a robot back and forth in the ${?} direction
while randomly providing an angular velocity about the ${?}-axis.

???PUBLISHERS:
  + ${topic_name} (${message_type}) - The velocity of an erratic turtle path

???SERVICES:
  + ${topic_name} (${service_type}) - Position of the new turtle

PARAMETERS:
  + frequency (double) - Velocity driving the robot

"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty # Imported the service std_srvs/srv/Empty
from rcl_interfaces.msg import ParameterDescriptor
from enum import Enum
from turtle_interfaces.srv import Waypoints
import math

# Enum class that indicates the different states of the node: MOVING, STOPPED
class state(Enum):
    MOVING = 0
    STOPPED = 1

# The Waypoint class that is a publisher node
class Waypoint(Node):

    def __init__(self):
        super().__init__('waypoint')
        # Initialize node state as STOPPED
        self.state = state.STOPPED
        # declare and get the frequency parameter and set default value
        self.declare_parameter("frequency", 100.0,
                               ParameterDescriptor(description="The frequency in which the msg is published"))
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        # Create service named toggle
        self.srv_1 = self.create_service(Empty, 'toggle', self.empty_callback)
        # Create service named load
        self.srv_2 = self.create_service(Waypoints, 'load', self.waypoints_callback)

        # Adjusted frequency for whatever the frequency param value is
        timer_period = 1.0/self.frequency  # seconds
        # create timer and timer callback for debug message issuing
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Issuing debug message
        # To run node in a mode that allows for viewing debug messages, run:
        # ros2 run turtle_control waypoint --ros-args -p frequency:=100.0 --log-level debug
        self.get_logger().debug('Issuing Command!')

    # Test the callback with the following command:
    # ros2 service call /toggle std_srvs/srv/Empty "{}"
    def empty_callback(self, request, response):
        self.get_logger().info('Incoming Request!')

        if self.state == state.MOVING:
            self.state = state.STOPPED
            self.get_logger().info('Stopping')

        elif self.state == state.STOPPED:
            self.state = state.MOVING
            self.get_logger().info('Moving')

        return response
    
    def waypoints_callback(self, request, response):
        # Temporarily hardcoding starting pos.
        pos_x = 10.0
        pos_y = 10.0
        distance = 0.0
        for i in request.points:
            diff = math.sqrt((pos_x - i.x)**2 + (pos_y - i.y)**2)
            distance += diff
        response.distance = distance
        return response



def main(args=None):
    rclpy.init(args=args)

    waypoint = Waypoint()

    rclpy.spin(waypoint)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waypoint.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()