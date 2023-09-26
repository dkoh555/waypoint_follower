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

# The Waypoint class that is a publisher node
class Waypoint(Node):

    def __init__(self):
        super().__init__('waypoint')
        # declare and get the frequency parameter and set default value
        self.declare_parameter("frequency", 100.0,
                               ParameterDescriptor(description="The frequency in which the msg is published"))
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        # Create service named toggle
        self.srv = self.create_service(Empty, 'toggle', self.empty_callback)
        # Adjusted frequency for whatever the frequency param value is
        timer_period = 1.0/self.frequency  # seconds
        # create timer and timer callback for debug message issuing
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Issuing debug message
        # To run node in a mode that allows for viewing debug messages, run:
        # ros2 run turtle_control waypoint --ros-args -p frequency:=100.0 --log-level debug
        self.get_logger().debug('Issuing Command!')

    def empty_callback(self, request, response):
        self.get_logger().info('Incoming request')


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