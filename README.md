# mci_turtlebot

## TODO
ROS node to save poses

## Navigation
1. create map
2. save map
3. move map.yaml & map.pgm to pi
4. start nav2 node on pi
5. profit?

```ros2 launch turtlebot4_navigation nav_bringup.launch.py slam:=off localization:=true map:=map_MCI.yaml```


## SSH
ssh ubuntu@192.168.243.136
turtlebot4

nmap -sn 192.168.1.0/24

## fix DDS
export CYCLONEDDS_URI=file://cycloneDDS.xml

## launch LiDAR
ros2 launch turtlebot4_bringup rplidar.launch.py
# stop lidar
ros2 service call /stop_motor std_srvs/srv/Empty {}
# start lidar
ros2 service call /start_motor std_srvs/srv/Empty {}

# rviz top view
ros2 launch turtlebot4_viz view_robot.launch.py

# save map
``
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'map_mci'"
```

# start nav2
ros2 launch turtlebot4_navigation nav_bringup.launch.py slam:=off localization:=true map:=map_mci.yaml


wlpassword013
/etc/netplan/50-cloud-init-yaml
