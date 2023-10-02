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
from turtlesim.srv import TeleportAbsolute, SetPen
from turtlesim.msg import Pose
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
        # Initalize variables
        self.pont_list = None
        self.state = state.STOPPED
        # This node will use Reentrant Callback Groups for nested services
        self.cbgroup = ReentrantCallbackGroup()
        # declare and get the frequency parameter and set default value
        self.declare_parameter("frequency", 100.0,
                               ParameterDescriptor(description="The frequency in which the msg is published"))
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value

        ###
        ### SERVICES
        ###
        # Create service named toggle
        self.srv_1 = self.create_service(Empty, 'toggle', self.empty_callback, callback_group=self.cbgroup)
        # Create service named load
        self.srv_2 = self.create_service(Waypoints, 'load', self.waypoints_callback, callback_group=self.cbgroup)

        ###
        ### CLIENTS
        ###
        # Create client for reseting the turtlesim
        self.cli_1 = self.create_client(Empty, 'reset', callback_group=self.cbgroup)
        while not self.cli_1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_reset = Empty.Request()
        # Create client for repositioning turtlesim
        self.cli_2 = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute', callback_group=self.cbgroup)
        while not self.cli_2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_repos = TeleportAbsolute.Request()
        # Create client for toggling the marker pen on/off
        self.cli_3 = self.create_client(SetPen, 'turtle1/set_pen', callback_group=self.cbgroup)
        while not self.cli_3.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_pen = SetPen.Request()

        ###
        ### SUBSCRIBERS
        ###
        # Create subscriber to get the current position of the turtle
        self.sub_pos = self.create_subscription(Pose, 'turtle1/pose', self.sub_pos_callback, 10)
        self.sub_pos


        ###
        ### TIMER
        ###
        # Adjusted frequency for whatever the frequency param value is
        timer_period = 1.0/self.frequency  # seconds
        # create timer and timer callback for debug message issuing
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.cbgroup)

    def timer_callback(self):
        # Issuing debug message
        # To run node in a mode that allows for viewing debug messages, run:
        # ros2 run turtle_control waypoint --ros-args -p frequency:=100.0 --log-level debug
        if self.state == state.MOVING:
            self.get_logger().debug('Issuing Command!')

    # Takes note of most recent x & y coord from the pose topic
    def sub_pos_callback(self, msg):
        self.recent_x = msg.x
        self.recent_y = msg.y

    # Test the callback with the following command:
    # ros2 service call /toggle std_srvs/srv/Empty "{}"
    def empty_callback(self, request, response):
        self.get_logger().info('Incoming Request!')

        if self.state == state.MOVING:
            self.state = state.STOPPED
            self.get_logger().info('Stopping')

        elif self.state == state.STOPPED:
            self.state = state.MOVING

        return response
    
    # When receives a number waypoints via service, it will then submit a client request
    # to reset turtlesim, then another to reposition the turtle to each waypoint
    # Example code to run for providing waypoints:
    # ros2 service call /load turtle_interfaces/srv/Waypoints "{points: [{x: 8.2, y: 5.0, z: 0.0}, {x: 4.0, y: 3.0, z: 0.0}, {x: 3.2, y: 9.4, z: 0.0}]}"
    async def waypoints_callback(self, request, response):
        # Total distance travelled
        distance = 0.0
        # Reset the turtle and turn its pen off immediately
        await self.cli_1.call_async(self.req_reset)
        self.get_logger().info('Reseting')
        await self.pen_off(True)
        # Taking note of turtle's initial reset coord
        orig_x = self.recent_x
        orig_y = self.recent_y
        pos_x = orig_x
        pos_y = orig_y
        # Move the turtle around, drawing the X's, and calculating the total distance for the entire 'cycle'
        for i in request.points:
            # Submit client request to move turtlesim to new position
            self.req_repos.x = i.x
            self.req_repos.y = i.y
            self.req_repos.theta = 0.0
            await self.cli_2.call_async(self.req_repos)
            self.get_logger().info('Repositioning')
            # Draw the X around the current position
            await self.draw_x(i.x, i.y)
            # Find the distance between the curr pos and the new position
            diff = math.sqrt((pos_x - i.x)**2 + (pos_y - i.y)**2)
            # Set the curr pos as the just now new position
            pos_x = i.x
            pos_y = i.y
            # Add to the total distance travelled
            distance += diff
        diff = math.sqrt((pos_x - orig_x)**2 + (pos_y - orig_y)**2) # Adding the distance from the last point to the starting one
        distance += diff
        # Repos the turtle to its first waypoint
        self.req_repos.x = request.points[0].x
        self.req_repos.y = request.points[0].y
        self.req_repos.theta = 0.0
        await self.cli_2.call_async(self.req_repos)
        # TEST THETA
        self.next_theta(self.recent_x, self.recent_y, request.points[1].x, request.points[1].y)
        # Set node state to STOPPED
        self.state = state.MOVING
        # Responds to client with total distance of waypoints
        response.distance = distance
        return response
    
    # Draws an X around the provided coordinate by calling the repositioning and set pen services
    async def draw_x(self, x_origin, y_origin):
        # Initialize an array of the coord of each vertices of the X
        size = 0.35
        x_vert_array = [x_origin-size, x_origin+size, x_origin-size, x_origin+size]
        y_vert_array = [y_origin-size, y_origin-size, y_origin+size, y_origin+size]
        # Turn the pen on and go to each vertices
        await self.pen_off(False)
        for i in range(4):
            self.req_repos.x = x_vert_array[i]
            self.req_repos.y = y_vert_array[i]
            self.req_repos.theta = 0.0
            await self.cli_2.call_async(self.req_repos)
            self.req_repos.x = x_origin
            self.req_repos.y = y_origin
            self.req_repos.theta = 0.0
            await self.cli_2.call_async(self.req_repos)
        # Turn the pen off
        await self.pen_off(True)
        
    ###
    ### TURTLE WAYPOINT NAVIGATION
    ###
    # Function that returns the correct theta (/pose) from starting waypoint to the target
    # 0.0 theta means the turtle is facing westward/the right
    def next_theta(self, start_x, start_y, end_x, end_y):
        delt_x = end_x - start_x
        delt_y = end_y - start_y
        theta = math.atan2(delt_y, delt_x)
        self.get_logger().info('theta: "%s"' % theta)
        return theta

    # Calls a service to toggle the pen of the turtle on/off
    # To turn the pen off, you can run the following command:
    # ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 255, b: 255, width: 3, 'off': 1}"
    async def pen_off(self, pen_off):
        self.req_pen.r = 255
        self.req_pen.g = 255
        self.req_pen.b = 255
        self.req_pen.width = 3
        self.req_pen.off = pen_off
        await self.cli_3.call_async(self.req_pen)

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