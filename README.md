# mci_turtlebot

## Things to try
- chrony
- delay in laserscan transforms
- delay during laser info processing?

## TODO
- reliable and reproducable navigation
- elevator control node
- GUI
- dropoff/pickup station docking
- task planner
- traffic planner(?)
- QoS level subscriber? (10 = REALIABILITY?)

## SSH
```
ssh ubuntu@<clientIP>
pw: turtlebot4
```

## WIFI
### Robotics Lab Wifi
```
SSID: ZentrumPRA
pw: wlpassword013
```
### Wifi setup file PI4
```
/etc/netplan/50-cloud-init-yaml
```
### find PI4 IP on network
```
nmap -sn 192.168.1.0/24
```

# Commands

## SLAM
### sync SLAM
```
ros2 launch turtlebot4_navigation slam_sync.launch.py
```
### save map
```
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'map_mci'"
```

## Navigation
1. create map
2. save map
3. move map.yaml & map.pgm to pi
4. start nav2 node on pi
```
ros2 launch turtlebot4_navigation nav_bringup.launch.py slam:=off localization:=true map:=map_MCI.yaml
```
5. profit?

## fix DDS
```
export CYCLONEDDS_URI=file://cycloneDDS.xml
```

## LiDAR
### launch
```
ros2 launch turtlebot4_bringup rplidar.launch.py
```
### stop lidar
```
ros2 service call /stop_motor std_srvs/srv/Empty {}
```
### start lidar
```
ros2 service call /start_motor std_srvs/srv/Empty {}
```

## RVIZ2
```
ros2 launch turtlebot4_viz view_robot.launch.py
```
