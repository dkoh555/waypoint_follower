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
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from enum import Enum
from turtle_interfaces.srv import Waypoints
from turtle_interfaces.msg import ErrorMetric
from turtlesim.srv import TeleportAbsolute, SetPen
from turtlesim.msg import Pose
import math
import asyncio
from rclpy.callback_groups import ReentrantCallbackGroup

# Enum class that indicates the different states of the node: MOVING, STOPPED
class state(Enum):
    MOVING = 0
    STOPPED = 1

# Enum class that indicates the different modes of the turtle: ROTATING, TRANSLATING, REACHED
class mode(Enum):
    ROTATING = 0
    TRANSLATING = 1
    REACHED = 2

# The Waypoint class that is a publisher node
class Waypoint(Node):

    def __init__(self):
        super().__init__('waypoint')
        # Initalize variables
        self.init_var()
        # This node will use Reentrant Callback Groups for nested services
        self.cbgroup = ReentrantCallbackGroup()

        ###
        ### Parameters
        ###
        # Declare and get the frequency parameter and set default value
        self.declare_parameter("frequency", 100.0,
                               ParameterDescriptor(description="The frequency in which the msg is published"))
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        # Declare and get the tolerance (for waypoint proximity) parameter and set the default value
        self.declare_parameter("tolerance", 0.5,
                               ParameterDescriptor(description="The proximity in which the turtle needs to be to a waypoint to classify as arrived"))
        self.tolerance = self.get_parameter("tolerance").get_parameter_value().double_value

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
        ### PUBLISHERS
        ###
        # Create publisher to move the turtle
        self.pub_cmdvel = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        # Create publisher to send out ErrorMetric messages
        self.pub_err = self.create_publisher(ErrorMetric, '/loop_metrics', 10)

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

    def init_var(self):
        self.recent_x = 0.0
        self.recent_y = 0.0
        self.point_list = None
        self.target_point_index = 1
        self.target_theta = None
        self.state = state.STOPPED
        self.turtle_mode = mode.REACHED
        self.pred_distance = 0.0
        self.actual_distance = 0.0
        self.complete_loops = 0

    async def timer_callback(self):
        
        move_msg = Twist()

        # If the node is in the MOVING state, move the turtle to the next waypoint
        if self.state == state.MOVING:
            # Issuing debug message
            # To run node in a mode that allows for viewing debug messages, run:
            # ros2 run turtle_control waypoint --ros-args -p frequency:=100.0 --log-level debug
            self.get_logger().debug('Issuing Command!')

            # If the new index is larger than the number of waypoints provided, reset back to 0
            # AKA when the turtle complete a cyle of waypoints, repeat it
            if self.target_point_index == len(self.point_list):
                self.target_point_index = 0
                self.complete_loops += 1

            # As the turtle is moving, keep track of its distance travelled
            self.track_distance()    

            # From waypoints list, obtain target point and navigate towards it
            target_point = self.point_list[self.target_point_index]
            # Get the Twist message to move the turtle to the target waypoint
            move_msg = await self.navigate_turtle(move_msg, target_point)

        # If the node is in the STOPPED state, keep it in place
        elif self.state == state.STOPPED:
            move_msg = self.move_turtle(move_msg, 0.0)

        self.pub_cmdvel.publish(move_msg)


    # Takes note of most recent x & y coord from the pose topic
    # And adds the difference between current and new coord to the acutal distance travelled
    def sub_pos_callback(self, msg):
        self.old_x = self.recent_x
        self.old_y = self.recent_y
        self.recent_x = msg.x
        self.recent_y = msg.y
        self.recent_theta = msg.theta

    # Test the callback with the following command:
    # ros2 service call /toggle std_srvs/srv/Empty "{}"
    def empty_callback(self, request, response):
        self.get_logger().info('Incoming Request!')

        if self.state == state.MOVING:
            self.state = state.STOPPED
            self.get_logger().info('Stopping')

        elif self.state == state.STOPPED:
            # If no waypoints are loaded, then log an error message and keep node in STOPPED state
            if self.point_list == None:
                self.get_logger().error('No waypoints loaded. Load them with the "load" service.')
                self.get_logger().error('Staying in STOPPED state.')
                self.state = state.STOPPED
            elif len(self.point_list) == 1:
                self.get_logger().error('Only one waypoints loaded. Unable to move to different waypoints.')
                self.get_logger().error('Staying in STOPPED state.')
                self.state = state.STOPPED
            else:
                self.state = state.MOVING

        return response
    
    # When receives a number waypoints via service, it will then submit a client request
    # to reset turtlesim, then another to reposition the turtle to each waypoint
    # Example code to run for providing waypoints:
    # ros2 service call /load turtle_interfaces/srv/Waypoints "{points: [{x: 8.2, y: 5.0, z: 0.0}, {x: 4.0, y: 3.0, z: 0.0}, {x: 3.2, y: 9.4, z: 0.0}]}"
    async def waypoints_callback(self, request, response):
        # Reinitialize variables
        self.init_var()
        # Save the list of waypoints
        self.point_list = request.points
        self.get_logger().info('Loading waypoints')
        # Total predicted distance to travel the entire cycle of waypoints
        self.pred_distance = 0.0
        # Reset the turtle and turn its pen off immediately
        await self.cli_1.call_async(self.req_reset)
        await self.pen_off(True)
        # Move the turtle around, drawing the X's, and calculating the total distance for the entire 'cycle'
        is_initial_point = True
        for i in request.points:
            # Submit client request to move turtlesim to new position
            self.req_repos.x = i.x
            self.req_repos.y = i.y
            self.req_repos.theta = 0.0
            await self.cli_2.call_async(self.req_repos)
            # If it's the initial/starting waypoint, take special note of its coord
            if is_initial_point:
                orig_x = self.recent_x
                orig_y = self.recent_y
                pos_x = orig_x
                pos_y = orig_y
                is_initial_point = False
            # Draw the X around the current position
            await self.draw_x(i.x, i.y)
            # Find the distance between the curr pos and the new position
            diff = math.sqrt((pos_x - i.x)**2 + (pos_y - i.y)**2)
            # Set the curr pos as the just now new position
            pos_x = i.x
            pos_y = i.y
            # Add to the total distance travelled
            self.pred_distance += diff
        diff = math.sqrt((pos_x - orig_x)**2 + (pos_y - orig_y)**2) # Adding the distance from the last point to the starting one
        self.pred_distance += diff
        # Repos the turtle to its first waypoint
        self.req_repos.x = request.points[0].x
        self.req_repos.y = request.points[0].y
        self.req_repos.theta = 0.0
        await self.cli_2.call_async(self.req_repos)
        # Set node state to STOPPED
        self.state = state.STOPPED
        # Responds to client with total distance of waypoints
        response.distance = self.pred_distance
        return response
    
    ###
    ### WAYPOINT MARKING
    ###
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
        
    ###
    ### TURTLE WAYPOINT NAVIGATION
    ###
    # Function that returns the correct theta (/pose) from starting waypoint to the target
    # 0.0 theta means the turtle is facing westward/the right
    def next_theta(self, start_x, start_y, end_x, end_y):
        delt_x = end_x - start_x
        delt_y = end_y - start_y
        theta = math.atan2(delt_y, delt_x)
        return theta
    
    # Function that returns a bool on whether the a given coord is within a certain radius of another given coord
    def is_near(self, start_x, start_y, end_x, end_y, rad):
        dist = math.sqrt((start_x - end_x)**2 + (start_y - end_y)**2)
        return dist <= rad
    
    # Function that returns a Twist message for moving the turtle forward
    def move_turtle(self, twist_msg, for_vel=2.0):
        new_msg = twist_msg
        new_msg.linear.x = for_vel
        return new_msg
    
    # Function that returns a boolean stating whether it is best to rotate clockwise from a starting theta to reach a target theta
    def is_clockwise_best(self, theta_1, theta_2):
        diff_cw = (theta_1 - theta_2) % (2 * math.pi)
        diff_ccw = (theta_2 - theta_1) % (2 * math.pi)
        return diff_cw <= diff_ccw
    
    # Function that calls a service to adjust the pose of the turtle so that it rotates to a desired angle in its current pos
    async def rotate_turtle(self, target_theta):
        self.req_repos.x = self.recent_x
        self.req_repos.y = self.recent_y
        self.req_repos.theta = target_theta
        await self.cli_2.call_async(self.req_repos)

    # Returns a Twist message that will guide the turtle towards the target point,
    # whether it has reached, or is rotating or moving towards it
    async def navigate_turtle(self, twist_msg, point):
        new_msg = twist_msg
        
        # When the turtle is at a waypoint
        if self.turtle_mode == mode.REACHED:
            # Calculates the angle needed to face the target point
            self.target_theta = self.next_theta(self.recent_x, self.recent_y, point.x, point.y)
            # Switch to ROTATING mode
            self.turtle_mode = mode.ROTATING
            # If turtle just finished going through a cycle of waypoints,
            # publish ErrorMetric msg to the appropriate topic when that happens
            if self.target_point_index == 1 and self.complete_loops > 0:
                self.publish_errormsg()

        # When the turtle is rotating itself at the start point
        elif self.turtle_mode == mode.ROTATING:
            # Readjust turtle orientation to face next waypoint, then switch to TRANSLATING mode
            await self.rotate_turtle(self.target_theta)
            self.turtle_mode = mode.TRANSLATING

        # When the turtle is moving forward to the target point
        elif self.turtle_mode == mode.TRANSLATING:
            # If the turtle is near the target point, switch to REACHED mode
            if self.is_near(self.recent_x, self.recent_y, point.x, point.y, self.tolerance):
                self.turtle_mode = mode.REACHED
                # Change point index so that navigate_turtle can move towards the next point
                self.target_point_index += 1
            # Else, move forward towards the target point
            else:
                new_msg = self.move_turtle(new_msg)
        
        return new_msg
    
    ###
    ### ERROR METRIC MESSAGE
    ###
    # Function calculates the tracks the totla distance travelled by the turtle
    def track_distance(self):
        diff = math.sqrt((self.old_x - self.recent_x)**2 + (self.old_y - self.recent_y)**2)
        self.actual_distance += diff

    # Function returns the error between calculated straight line distance and actual distance travelled
    def error_distance(self):
        err = ((abs(self.actual_distance - self.pred_distance))/self.actual_distance) * 100
        return err

    # Function publishes the Error Metric on the appropriate topic
    # Also resets the actual distance travelled
    def publish_errormsg(self):
        msg = ErrorMetric()
        msg.complete_loops = self.complete_loops
        msg.actual_distance = self.actual_distance
        msg.error = self.error_distance()
        self.pub_err.publish(msg)
        self.actual_distance = 0.0
            

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