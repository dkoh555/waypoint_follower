# ME495 Embedded Systems Homework 1
Author: Damien Koh
1. Use `ros2 launch turtle_control waypoints.launch.xml` to run the code
2. The `ros2 service call /load turtle_interfaces/srv/Waypoints "{points: [{x: 1.5, y: 1.7, z: 0.0}, {x: 2.1, y: 9.5, z: 0.0}, {x: 7.1, y: 6.0, z: 0.0}, {x: 4.1, y: 2.5, z: 0.0}, {x: 8.1, y: 1.4, z: 0.0}, {x: 4.1, y: 5.2, z: 0.0}]}"` service loads waypoints for the turtle to follow.
3. The `ros2 service call /toggle std_srvs/srv/Empty "{}"` starts and stops the turtle.
4. Here is a video of the turtle in action.
   `[me495_hw1_example_run.webm](https://github.com/ME495-EmbeddedSystems/homework1-dkoh555/assets/107823507/c1f3796f-a085-4f19-acd3-dff14bcf33b4)`

## Additional Waypoint Node Commands:
1. To turn the pen on or off run `ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 255, b: 255, width: 3, 'off': 1}"`
2. To load a different set of waypoints for the turtle to follow, run `ros2 service call /load turtle_interfaces/srv/Waypoints "{points: [{x: 8.2, y: 5.0, z: 0.0}, {x: 4.0, y: 3.0, z: 0.0}, {x: 3.2, y: 9.4, z: 0.0}]}"`
3. To run seperately run the waypoint node in a mode that allows for viewing debug messages, run `ros2 run turtle_control waypoint --ros-args -p frequency:=100.0 --log-level debug`