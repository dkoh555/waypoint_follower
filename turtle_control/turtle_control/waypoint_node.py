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

class state(Enum):
    """ Current state of the system (waypoint node).
        Determines what movement commands are published to the turtle,
        whether it is MOVING or STOPPED
    """
    MOVING = 0
    STOPPED = 1

class mode(Enum):
    """ Current stage the turtle should be in to move to the next waypoint.
        Determines what movement commands are published to the turtle,
        whether it is ROTATING, TRANSLATING, or REACHED
    """
    ROTATING = 0
    TRANSLATING = 1
    REACHED = 2

class Waypoint(Node):
    """ Publishes geometry_msgs/Twist commands at a fixed rate for the turtle
    """
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
        # Create client for resetting the turtlesim
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
        """ Initializes all of the waypoint node's variables.
        """
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

    def timer_callback(self):
        """ Timer callback for the waypoint node.

            Depending on whether the node state is MOVING or STOPPED, provides a geometry_msgs/Twist
            that moves the turtle through its different provided waypoints or keeps it in place respectively
        """
        move_msg = Twist()
        # If the node is in the MOVING state, move the turtle to the next waypoint
        if self.state == state.MOVING:
            self.get_logger().debug('Issuing Command!')

            # If the new index is larger than the number of waypoints provided, reset back to 0
            # AKA when the turtle complete a cyle of waypoints, repeat it
            if self.target_point_index == len(self.point_list):
                self.target_point_index = 0
                self.complete_loops += 1

            # From waypoints list, obtain target point and navigate towards it
            target_point = self.point_list[self.target_point_index]
            # Get the Twist message to move the turtle to the target waypoint
            move_msg = self.navigate_turtle(move_msg, target_point)

        # If the node is in the STOPPED state, keep it in place
        elif self.state == state.STOPPED:
            move_msg = self.move_turtle(move_msg, 0.0)

        self.pub_cmdvel.publish(move_msg)


    def sub_pos_callback(self, msg):
        """ Callback function for the turtle1/pose topic.

            Receives and takes note of the turtle's current position,
            also tracks the distance travelled by the turtles when the nodes is in the MOVING state
            
            Args:
                msg (turtlesim/Pose): A message that contains a Pose message, containing the
                    current x, y, theta, and linear and angular velocities of the turtle
        """
        self.old_x = self.recent_x
        self.old_y = self.recent_y
        self.recent_x = msg.x
        self.recent_y = msg.y

        # If node is in MOVING state, then add diff between old and new position to distance travelled
        if self.state == state.MOVING:
            diff = math.sqrt((self.old_x - self.recent_x)**2 + (self.old_y - self.recent_y)**2)
            self.actual_distance += diff

        self.recent_theta = msg.theta

    def empty_callback(self, request, response):
        """ Callback function for the toggle service.

            When provided with a std_srvs/Empty message,
            the node will switch between MOVING and STOPPED states
            
            Args:
                request (Empty): A message that contains nothing

                response (Empty): The response object

            Returns:
                Empty: Contains nothing
        """
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
    
    async def waypoints_callback(self, request, response):
        """ Callback function for the load service.

            When provided with an array of geometry_msgs/Points, the turtle will 'draw' X's at each point,
            reposition itself at the first point in said array, and then returns the predicted distance the
            turtle would travel to go through one cycle of the provided list of points.
            
            Args:
                request (geometry_msgs/Point[]): An array containing geometry_msgs/Point,
                    which then contain x, y, z fields that correspond with coordinates of
                    each point - because the functions use 2D coords the z field is ignored

                response (float64): The response object

            Returns:
                float64: The predicted straight-line distance the turtle would travel to
                    go through a cycle of the given points
        """
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
                orig_x = i.x
                orig_y = i.y
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
    ### WAYPOINT MARKING FUNCTIONS
    ###

    async def draw_x(self, x_origin, y_origin):
        """ Service client for the turtle1/teleport_absolute service that uses 
            the turtle to 'draw' and X at a given set of coordinates.

            Args:
                x_origin (float): x-coord of the X's centerpoint
                y_origin (float): y-coord of the X's centerpoint
        """
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

    async def pen_off(self, pen_off):
        """ Service client for the turtle1/set_pen service that turns the pen on/off depending on the function input.

            Args:
                pen_off (boolean): Whether the pen should turned off
        """
        self.req_pen.r = 255
        self.req_pen.g = 255
        self.req_pen.b = 255
        self.req_pen.width = 3
        self.req_pen.off = pen_off
        await self.cli_3.call_async(self.req_pen)
        
    ###
    ### TURTLE WAYPOINT NAVIGATION FUNCTIONS
    ###

    def next_theta(self, start_x, start_y, end_x, end_y):
        """ Returns a float that indicates the angle a given end point is relative to a given starting point.
            (This value is within [-pi, pi])

            Args:
                start_x (float): The current x-coord
                start_y (float): The current y-coord
                end_x (float): The target x-coord
                end_y (float): The target y-coord
            
            Returns:
                Float: The angle between the starting and target points are
        """
        delt_x = end_x - start_x
        delt_y = end_y - start_y
        theta = math.atan2(delt_y, delt_x)
        return theta
    
    def is_near(self, start_x, start_y, end_x, end_y, rad):
        """ Returns a boolean that indicates if a set of given 2D coordinates are within a given radius of another set of coordinates.

            Args:
                start_x (float): The current x-coord
                start_y (float): The current y-coord
                end_x (float): The target x-coord
                end_y (float): The target y-coord
                rad (float): The radius the two points have to be within of each other to be marked as 'near' each other
            
            Returns:
                Bool: States whether the two points are near each other is True/False
        """
        dist = math.sqrt((start_x - end_x)**2 + (start_y - end_y)**2)
        return dist <= rad
    
    def move_turtle(self, twist_msg, for_vel=2.0):
        """ Returns a geometry_msgs/Twist message that moves the turtle forward at a given velocity.

            Args:
                twist_msg (geometry_msgs/Twist): The initial movement command of the turtle, corresponding with linear & angular velocity
                for_vel (float): The desired linear velocity of the turtle
            
            Returns:
                geometry_msgs/Twist: The new movement command for the turtle to move towards the target waypoint
        """
        new_msg = twist_msg
        new_msg.linear.x = for_vel
        return new_msg
    
    def is_clockwise_best(self, theta_1, theta_2):
        """ Returns a boolean that indicates if it's faster to rotate clockwise to reach a given target theta from given starting theta.

            Args:
                theta_1 (float): The initial theta
                theta_2 (float): The target theta
            
            Returns:
                Bool: States whether moving clockwise is the fastest path is True/False
        """
        diff_cw = (theta_1 - theta_2) % (2 * math.pi)
        diff_ccw = (theta_2 - theta_1) % (2 * math.pi)
        return diff_cw <= diff_ccw
    
    def rotate_turtle(self, twist_msg, ang_vel=0.5):
        """ Returns a geometry_msgs/Twist message that rotates the turtle at a given angular velocity.

            Args:
                twist_msg (geometry_msgs/Twist): The initial movement command of the turtle, corresponding with linear & angular velocity
                ang_vel (float): The desired angular velocity of the turtle
            
            Returns:
                geometry_msgs/Twist: The new movement command for the turtle to move towards the target waypoint
        """
        new_msg = twist_msg
        new_msg.angular.z = ang_vel
        return new_msg

    def navigate_turtle(self, twist_msg, point):
        """ Returns geometry_msgs/Twist message to guide turtle to a given waypoint from its current position.

            Args:
                twist_msg (geometry_msgs/Twist): The initial movement command of the turtle, corresponding with linear & angular velocity
                point (geometry_msgs/Point): The current target waypoint the turtle is moving towards
            
            Returns:
                geometry_msgs/Twist: The new movement command for the turtle to move towards the target waypoint
        """
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
            # If the turtle is facing the target point (or close enough to that), switch to TRANSLATING mode
            if abs(self.recent_theta - self.target_theta) <= 0.0005:
                self.turtle_mode = mode.TRANSLATING
            # Else, rotate turtle towards target point
            else:
                ang_vel = 4.0
                if self.is_clockwise_best(self.recent_theta, self.target_theta):
                    ang_vel = ang_vel * -1.0
                
                # Adjust the angular velocity based on how close the turtle is to target theta,
                # the closer it is the slower it turns for a more accurate orientation
                diff_theta = abs((self.target_theta - self.recent_theta + math.pi) % (2 * math.pi) - math.pi)
                prop_diff = diff_theta/math.pi # If the diff is pi, then that would be max/original speed
                ang_vel *= prop_diff

                new_msg = self.rotate_turtle(new_msg, ang_vel)

        # When the turtle is moving forward to the target point
        elif self.turtle_mode == mode.TRANSLATING:
            # If the turtle is near the target point, switch to REACHED mode
            if self.is_near(self.recent_x, self.recent_y, point.x, point.y, self.tolerance):
                self.turtle_mode = mode.REACHED
                # Change point index so that navigate_turtle can move towards the next point
                self.target_point_index += 1
            # Else, move forward towards the target point
            else:
                for_vel = 4.0

                # Adjust the forward velocity based on how close the turtle is to target position,
                # the closer it is the slower it moves for a more arrival
                diff_pos = math.sqrt((point.x - self.recent_x)**2 + (point.y - self.recent_y)**2)
                if diff_pos < 1.0:
                    for_vel *= diff_pos
                
                new_msg = self.move_turtle(new_msg, for_vel)
        
        return new_msg
    
    ###
    ### ERROR METRIC MESSAGE FUNCTIONS
    ###

    def error_distance(self):
        """ Returns the percentage error between actual distance travelled and predicted distance.
        """
        err = ((abs(self.actual_distance - self.pred_distance))/self.actual_distance) * 100
        return err

    def publish_errormsg(self):
        """ Publishes turtle_interfaces/ErrorMetric message when called.

            Reports the number of cycles of the waypoints completed by the turtle,
            the distance travelled by the turtle, and the percentage error between
            distance travelled and predicted distance to be travelled.
        """
        msg = ErrorMetric()
        msg.complete_loops = self.complete_loops
        msg.actual_distance = self.actual_distance
        msg.error = self.error_distance()
        self.pub_err.publish(msg)
        self.pred_distance += self.pred_distance/self.complete_loops
            

def main(args=None):
    """ The main() function.
    """
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