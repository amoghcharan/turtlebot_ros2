# turtlebot_ros2
Using Turtlebot 3 and ROS 2 (Foxy) for different robotics applications. The directories provide code to accomplish the following:

| Directory | Description |
| --- | ----------- |
| dead_reckoning_obstacle_avoidance | Utilizing onboard Odometry and LiDAR to accomplish dead reckoning path following and obstacle detection and avoidance, respectively. |
| image_detection_follower | Detecting circular targets with Hough Circles and utilizing Rasberry Pi camera and onboard LiDAR to follow and reach the position of the target, static or dynamic. |
| machine_learning_path_follower | Using K-Nearest Neighbors Machine Learning Model to detect directional signs and building on dead reckoning path follower to navigate an obstacle course. |
| navigation2 | Leveraging ROS Navigation 2 stack with A* algorithm and Pure Pursuit Controller to avoid obstacles and arrive at predefined waypoints. |