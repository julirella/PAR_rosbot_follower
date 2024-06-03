# PAR_rosbot_follower

## Commands

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