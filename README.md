# PAR_rosbot_follower

## Commands

### Lidar Logger
Start node:
```
ros2 run rosbot_follower lidar_logger
```

to get a reading open a second terminal:
```
ros2 topic pub -1 /reading std_msgs/msg/Bool "data: true"
```