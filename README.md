# Person Tracking

ROS package to detect and follow nearby objects.

The package is based on the 3D lidar and uses euclidean clustering and tracking on pointclouds to find and follow the nearest object.

### Usage
Set the following parameters in `person_tracker.launch`:
  - `lidar_topic (defualt: "/scan")`: Topic where lidar pointclouds are published (type: *sensor_msgs/PointCloud2*)
  - `tracking_topic (default: "/start_tracking")`: Topic where commands are published to start/stop tracking (type: *std_msgs/Bool*)
  - `cmd_vel_topic (default: "/cmd_vel")`: Topic where velocity commands should be published (type: *geometry_msgs/Twist*)
  - `keep_points_upto (default: 10.0)`: Maximum distance around to bot in which obstacles should be searched
  - `distance_to_maintain (default: 1.0)`: Distance to maintain from the target at all times
  - `linear_threshold (default: 0.1)`: Minimum linear displacement from the target for bot to start moving
  - `angular_threshold (default: 0.1)`: Minimum angular displacement from the target for bot to start moving
  - `max_linear_speed (default: 0.5)`: Maximum linear speed for the robot
  - `min_linear_speed (default: 0.1)`: Minimum linear speed for the robot
  - `max_angular_speed (default: 1.0)`: Maximum angular speed for the robot
  - `min_angular_speed (default: 0.0)`: Minimum angular speed for the robot
  - `time_to_x (default: 1.0)`: Time it would take to reach the goal if the max velocity permits. Used for scaling linear velocity.
  - `time_to_angle (default: 1.0)`: Time it would take to reach the goal if the max velocity permits. Used for scaling angular velocity.
  - `clip_cloud (default: true)`: Clip the pointcloud to look for objects only in a specified angular range when tracking is off.
  - `clip_angle_min (default: -1.31)`: Minimum clipping angle (radian)
  - `clip_angle_max (default: 1.31)`: Maximum clipping angle (radian)

To launch the person tracking node:
```bash
roslaunch person_tracking person_tracker.launch
```
