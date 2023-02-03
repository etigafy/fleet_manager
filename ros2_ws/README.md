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



