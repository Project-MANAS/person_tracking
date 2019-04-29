# Person Tracking

ROS package to detect and follow nearby objects.

The package is based on the 3D lidar and uses euclidean clustering and tracking on pointclouds to find and follow the nearest object.

### Usage
Set the following parameters in `person_tracker.launch`:
  - `lidar_topic (defualt: "/scan")`: Topic where lidar pointclouds are published (type: *sensor_msgs/PointCloud2*)
  - `tracking_topic (default: "/start_tracking")`: Topic where commands are published to start/stop tracking (type: *std_msgs/Bool*)
  - `keep_points_upto (default: 10.0)`: Maximum distance around to bot in which obstacles should be searched
  - `distance_to_maintain (default: 1.0)`: Distance to maintain from the target at all times
  - `x_threshold (default: 0.1)`: X-Displacement from the target for bot to start publishing movement commands
  - `y_threshold (default: 0.1)`: Y-Displacement from the target for bot to start publishing movement commands
  - `x_scale (default: 1.0)`: How much to scale the x-displacement to calculate linear velocity
  - `y_scale (default: 2.5)`: How much to scale the y-displacement to calculate angular velocity
  - `max_linear_speed (default: 0.5)`: Maximum linear speed for the robot
  - `min_linear_speed (default: 0.1)`: Minimum linear speed for the robot
  - `max_angular_speed (default: 1.0)`: Maximum angular speed for the robot
  - `min_angular_speed (default: 0.0)`: Minimum angular speed for the robot

To launch the person tracking node:
```bash
roslaunch person_tracking person_tracker.launch
```
