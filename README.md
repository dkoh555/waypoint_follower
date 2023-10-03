# ME495 Embedded Systems Homework 1
Author: ${Your Name}
1. Use `ros2 launch turtle_control waypoints.launch.xml` to run the code
2. The `ros2 service call /load turtle_interfaces/srv/Waypoints "{points: [{x: 8.2, y: 5.0, z: 0.0}, {x: 4.0, y: 3.0, z: 0.0}, {x: 3.2, y: 9.4, z: 0.0}]}"` service loads waypoints for the turtle to follow.
3. The `ros2 service call /toggle std_srvs/srv/Empty "{}"` starts and stops the turtle.
4. Here is a video of the turtle in action.
   `${embed video here}`