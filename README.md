# Turtlebot 4 Resource Collection

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

### fix oak-d: no available devices
```
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules

sudo udevadm control --reload-rules && sudo udevadm trigger
```
