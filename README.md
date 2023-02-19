# Turtlebot 4 Resource Collection

## TODO
- reliable and reproducable navigation &#10004;
- elevator control node &#10004;
- GUI &#10004;
- dropoff/pickup station docking
- task planner &#10004;
- traffic planner
- QoS level subscriber &#10004;

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
nmap -sn 192.168.<subnet>.0/24
```

## MISC
### fix oak-d: no available devices
```
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules

sudo udevadm control --reload-rules && sudo udevadm trigger
```
