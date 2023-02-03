# Elevator

## Server starten auf Turtlebot

#### Server starten
````
ros2 run tt_cpp elevator_server
````
Known issues:
- Safety check: wenn Endstop bei Start ausgelöst: Fehlermeldung

#### (Optional) Client starten
Goal vorgeben
Parameter: Position (mm), Velocity (mm/s), Acceleration (mm/s2)
````
ros2 run tt_cpp elevator_client 150 16 16
````
# POIs einlernen

#### Navigation mit Map starten
````
ros2 launch turtlebot4_navigation nav_bringup.launch.py slam:=off localization:=true map:=map_MCI.yaml
````

#### Pose Einlern RVIZ
````
ros2 launch tt_cpp orientation_logger_launch.py 
````

- Initial Pose einlernen (linker "2D Pose Estimate")
- Alle POIs einlernen (rechter "2D Pose Estimate")
  - POIs gespeichert in "waypoints_formatted.dat"
    - Machine Pickup/DropOff
    - CharginStation (pro Roboter)
    - PickupStation/DropOffStation
    - IdleStations (pro Roboter)
- Copy folder ros2_ws/put_in_home/taskPlannerData to ~/taskPlannerData
- POIs aus "waypoints_formatted.dat" in entsprechende Dateien in taskPlannerData kopieren
  - Wichtig: IDs vergeben und Gesamtzahl anpassen
    Example:
    ````
    num_machines=2
    #########################
    id=1
    pickup_x(mm)=2.651624
    pickup_y(mm)=0.198512
    pickup_theta(degrees)=0.403118
    processing_time(ms)=10000
    #########################
    id=2
    pickup_x(mm)=2.519932
    pickup_y(mm)=-1.221245
    pickup_theta(degrees)=-1.147145
    processing_time(ms)=7000
    #########################
    ````

Known issues:
- Pfad `/home/h/waypoints_formatted.dat` ist hardcoded
  - Workaround: Symlink to own home folder:
  ````
  sudo ln -s /home/[username] /home/h
  ````

