# CRAZY TURTLE
Demonstration package for ME495.
This README is intentionally vague.
Figuring out how this package works and filling in the details is part of the
exercise. Replace the blanks marked with `${ITEM}` with your answer.
Keep the backticks but remove the `${}`, so `${ITEM}` becomes `My Answer`.
Unless otherwise specified, list the command and all arguments that you passed to it.

## Repository Configuration
1. The `crazy_turtle` git repository consists of the ROS 2 packages `crazy_turtle` and `crazy_turtle_interfaces`.
2. The package `crazy_turtle` is a `python` package.
2. The package `crazy_turtle_interfaces` is a `cmake` package.


## Setup Instructions
1. Build the workspace using `colcon build` so that it is unnecessary to rebuild when python files change.
2. Initialize the ROS environment (i.e., set the necessary ROS environment variables) by executing `source /opt/ros/iron/setup.bash`
3. Make sure no other ROS nodes are running prior to starting by inspecting the results of `ros2 node list`.
3. Run the launchfile `go_crazy_turtle.launch.xml` by executing `ros2 launch crazy_turtle go_crazy_turtle.launch.xml`
4. When running you can see a visual depiction of the ROS graph using the `rqt_graph` command.
   The ROS graph, including all topics and node labels, looks like:
   ![rosgraph](https://github.com/ME495-EmbeddedSystems/homework1-dkoh555/assets/107823507/9e913b66-cbcf-4efb-b353-ec7885740408)

## Runtime Information
The `launchfile` from above should be running at all times when executing these commands.
If the nodes launched from the `launchfile` are not running, you will get incorrect results.

5. Use the ROS command `ros2 node list` to list all the nodes that are running.
   The output of the command looks like
   ```
   damien@damien-xps:~/me495/hw_ros_ws$ ros2 node list
   /mover
   /roving_turtle
   ```
6. Use the ROS command `ros2 topic list` to list the topics
   The output of the command looks like
   ```
   damien@damien-xps:~/me495/hw_ros_ws$ ros2 topic list
   /parameter_events
   /rosout
   /turtle1/cmd_vel
   /turtle1/color_sensor
   /turtle1/pose
   ```

7. Use the ROS command `ros2 topic hz /turtle1/cmd_vel` to verify that the frequency of
   the `/turtle1/cmd_vel` topic is `120 Hz`

8. Use the ROS command `ros2 service list` to list the services.
   The output of the command looks like
   ```
   damien@damien-xps:~/me495/hw_ros_ws$ ros2 service list
   /clear
   /kill
   /mover/describe_parameters
   /mover/get_parameter_types
   /mover/get_parameters
   /mover/list_parameters
   /mover/set_parameters
   /mover/set_parameters_atomically
   /reset
   /roving_turtle/describe_parameters
   /roving_turtle/get_parameter_types
   /roving_turtle/get_parameters
   /roving_turtle/list_parameters
   /roving_turtle/set_parameters
   /roving_turtle/set_parameters_atomically
   /spawn
   /switch
   /turtle1/set_pen
   /turtle1/teleport_absolute
   /turtle1/teleport_relative
   ```

9. Use the ROS command `ros2 service type /switch` to determine the type of the `/switch` service, which is `crazy_turtle_interfaces/srv/Switch`.

10. Use the ROS command `ros2 param list` to list the parameters of all running nodes
    ```
    damien@damien-xps:~/me495/hw_ros_ws$ ros2 param list
   /mover:
      use_sim_time
      velocity
   /roving_turtle:
      background_b
      background_g
      background_r
      holonomic
      qos_overrides./parameter_events.publisher.depth
      qos_overrides./parameter_events.publisher.durability
      qos_overrides./parameter_events.publisher.history
      qos_overrides./parameter_events.publisher.reliability
      use_sim_time

    ```

11. Use the ROS command `${command and args}` to get information about the `/mover` `velocity` parameter, including its type, description, and constraints
    ```
    ${full output of the command here}
    ```

12. Use the ROS command `${command and args}` to retrieve a template/prototype for entering parameters for the `/switch` service on the command line.
    ```
    ${full output of the command here}
    ```

## Package Exploration
1. Use the ROS command `${command and args}` to list the interface types defined by `crazy_turtle_interfaces`
   The output of the command looks like
   ```
   ${list service types here}
   ```
2. Use the ROS command `${command and args}` to list the executables included with the `crazy_turtle` package
   The output of the command looks like
   ```
   ${list executables here}
   ```

## Live Interaction
1. Use the command `${command and args here}` to retrieve the value of the `/mover velocity` parameter, which is `${value here}`.
2. The ROS command to call the `/switch` service, and it's output is listed below:
    ```
    ${enter the command and its output here. Call with x=1.0, y=2.0, theta=0.0, angular_velocity=3.0, linear_velocity=4.0}
    ```
3. The `switch` service performs the following actions (in sequence):
    1. It `${what does it do? kills | spawns | resets}` the current turtle
    2. It then respawns a new turtle at `${location as a function of the `/switch` service parameters}`
4. What happens to the turtle's motion if you use `${command and args here}` to change `/mover velocity` to 10? `${faster | slower | same}`
5. Use the Linux command `${command and args}` to kill the `/mover` node.
6. Use the ROS command `${command and args}` to start the `/mover` node with a velocity of 10. 
    - Be sure to remap `cmd_vel` to `/turtle1/cmd_vel`.
7. What happened to the turtle's velocity after relaunching `mover`? `${faster | slower | same}`
