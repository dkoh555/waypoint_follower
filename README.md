# ME495 Embedded Systems Homework 1
Author: Damien Koh
1. Use `ros2 launch turtle_control waypoints.launch.xml` to run the code
2. The `ros2 service call /load turtle_interfaces/srv/Waypoints "{points: [{x: 1.5, y: 1.7, z: 0.0}, {x: 2.1, y: 9.5, z: 0.0}, {x: 7.1, y: 6.0, z: 0.0}, {x: 4.1, y: 2.5, z: 0.0}, {x: 8.1, y: 1.4, z: 0.0}, {x: 4.1, y: 5.2, z: 0.0}]}"` service loads waypoints for the turtle to follow.
3. The `ros2 service call /toggle std_srvs/srv/Empty "{}"` starts and stops the turtle.
4. Here is a video of the turtle in action:
   https://github.com/ME495-EmbeddedSystems/homework1-dkoh555/assets/107823507/07853436-2cb8-48bf-8541-b0e316073c53
   [me495_hw1_example_run.webm](https://github.com/ME495-EmbeddedSystems/homework1-dkoh555/assets/107823507/3a7a1c17-2e27-4c2f-95a3-26f78f52ff1c)


## Additional Waypoint Node Commands:
1. To turn the pen on or off run `ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 255, b: 255, width: 3, 'off': 1}"`
2. To load a different set of waypoints for the turtle to follow, run `ros2 service call /load turtle_interfaces/srv/Waypoints "{points: [{x: 8.2, y: 5.0, z: 0.0}, {x: 4.0, y: 3.0, z: 0.0}, {x: 3.2, y: 9.4, z: 0.0}]}"`
3. To run seperately run the waypoint node in a mode that allows for viewing debug messages, run `ros2 run turtle_control waypoint --ros-args -p frequency:=100.0 --log-level debug`
