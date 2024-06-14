# PAR_rosbot_follower

## Commands

### Lidar Track
For every laser scan (this node is subscribed to the /scan topic), the lidar tracker checks whether any object in the scan matches the characteristics of the target rosbot (including the affixed cylinder). If found the angle from our follower to the target (in the lidar frame of the follower) is published to the follow/lidar_angle topic.

For testing you may simulate this behaviour using the command:
```
ros2 topic pub -1 /follow/lidar_angle std_msgs/msg/Float64 "data: 90.0"
```
in the above example we are simulating an object found at 90 degrees.

The lidar tracker also uses another topic /compute_lidar to toggle on the object detection feature, at time of writing this is by default on, though this may be used in future to save resources. To test this toggle, use the command:
```
ros2 topic pub -1 /compute_lidar std_msgs/msg/Bool "data: true"
```

### Lidar Logger
Start node:
```
ros2 run rosbot_follower lidar_logger
```

to prompt a reading open a second terminal:
```
ros2 topic pub -1 /reading std_msgs/msg/Bool "data: true"
```
note that it may also be useful to compare these finding with the rviz display:
```
rviz2 -d $AIIL_CHECKOUT_DIR/humble_workspace/src/aiil_rosbot_demo/rviz/rosbot-default.rviz
```