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
from turtlesim.srv import TeleportAbsolute
import math
import asyncio
from rclpy.callback_groups import ReentrantCallbackGroup

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
        # This node will use Reentrant Callback Groups for nested services
        self.cbgroup = ReentrantCallbackGroup()
        # declare and get the frequency parameter and set default value
        self.declare_parameter("frequency", 100.0,
                               ParameterDescriptor(description="The frequency in which the msg is published"))
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value

        # Create service named toggle
        self.srv_1 = self.create_service(Empty, 'toggle', self.empty_callback, callback_group=self.cbgroup)
        # Create service named load
        self.srv_2 = self.create_service(Waypoints, 'load', self.waypoints_callback, callback_group=self.cbgroup)

        # Create client for reseting the turtlesim
        self.cli_1 = self.create_client(Empty, 'reset')
        while not self.cli_1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_reset = Empty.Request()

        # Create client for repositioning turtlesim
        self.cli_2 = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute', callback_group=self.cbgroup) # when running from CLI will need to account for turtle name
        while not self.cli_2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_repos = TeleportAbsolute.Request()

        # Adjusted frequency for whatever the frequency param value is
        timer_period = 1.0/self.frequency  # seconds
        # create timer and timer callback for debug message issuing
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.cbgroup)

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
    
    # When receives a number waypoints via service, it will then submit a client request
    # to reset turtlesim, then another to reposition the turtle to each waypoint
    # Example code to run for providing waypoints:
    # ros2 service call /load turtle_interfaces/srv/Waypoints "{points: [{x: 8.2, y: 5.0, z: 0.0}, {x: 4.0, y: 3.0, z: 0.0}]}"
    async def waypoints_callback(self, request, response):
        # Temporarily hardcoding starting pos.
        pos_x = 10.0
        pos_y = 10.0
        # Total distance travelled
        distance = 0.0
        await self.cli_1.call_async(self.req_reset)
        self.get_logger().info('Reseting')
        # await rclpy.spin_until_future_complete(self, self.future)
        for i in request.points:
            # Submit client request to move turtlesim to new position
            self.req_repos.x = i.x
            self.req_repos.y = i.y
            self.req_repos.theta = 0.0
            await self.cli_2.call_async(self.req_repos)
            self.get_logger().info('Repositioning')
            # await rclpy.spin_until_future_complete(self, self.future)
            # Find the distance between the curr pos and the new position
            diff = math.sqrt((pos_x - i.x)**2 + (pos_y - i.y)**2)
            # Set the curr pos as the just now new position
            pos_x = i.x
            pos_y = i.y
            # Add to the total distance travelled
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