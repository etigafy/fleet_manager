# Fleet Management Executables - Overview
## task_planner
Main planning server that accepts and distributes tasks from the GUI or task_generator nodes and distributes them to clients running the turtle_client node. Can distribute tasks to multiple clients using different clientIDs.
The function of the task planner can be divided into two sub-groups:
- Client Management
- Task Management

### Client Management
Clients ping the planner periodically with information about battery, position and task state. If the server receives pings from a new client, the client is added to the list of clients (clientMap), and is flagged as online. If said pings stop for any reason, the server will classify the client to be timed out and the client is flagged as offline. Only online-clients are able to receive tasks from the server.

### Task Management
The planner is able to receive tasks using the /GuiService service. In fact, internally, a "task" is only the smallest entity (e.g.: go to point A and drop package) that is further encapsulated in a "task group" (e.g. pick package at location A and drop it at location B). This subdivide is required to ensure that a package cannot get stuck on a client, as a client that picks a package must also be the same client dropping it off. Furthermore, task groups are encapsulated by "deliveries" which is the internal term used for a procedure that picks a package from a pickup station, takes it to one or several machines, and drops the package off at a dropoff station. Every delivery must have this type of structure, i.e. declaring a dedicated pickup, dropoff, and x machines the package is passed through, where x may range from 0 to infinity (or more like 2<sup>32</sup>, I guess).

Deliveries requests are sent through the GUI or task_generator nodes. Deliveries received through either of the two channels trigger a callback_deliveryRequest(), where the delivery and its subtasks are validated and added to the delivery array (deliveryMap). The service client making the request will receive a response depending on the validity of the request.

Now, the task distributor will try to assign task groups to available clients. The client best fit for the job will receive the task group. If none are available, nothing happens.

This is also a good point in time to briefly touch upon the internal landmark management. As two clients cannot occupy the same space on a map, the task planner must ensure that there will be no interfering tasks. For this reason, the POI (Point Of Interest), further referred to as landmarks, occupation must also be handled by the planner. This is done by assigning both the new task group and the relevant landmarks of the subtasks to the selected client. This way, the planner prevents itself from assigning tasks with duplicate landmarks.

(BETA) Tasks assigned to a client, as well as relevant landmarks, are saved to the clients' "memory". This memory will persist through a client's offline period, as it is located on the server.

As the task memory and distribution is handled by the server, the client only acts as a slave that fulfills tasks issued by the task planner (master). After accepting a task issued by the server, the client goes into a BUSY-state, making it temporarily unavailable to task distribution. Once the client has completed the task, the client's next ping informs the server about the task's completion and the client is made available again, ready to receive a new task.

If there are no task groups in the queue, after a brief waiting period the client is marked as idle and sent to an available idle location (landmark).

Charging is handled by a upper and lower threshold. If the client passes the lower threshold, it will complete any outstanding tasks in its memory and subsequently generate a charging task. It will keep charging at least until it reached the upper charging threshold, at which point it will be available to receive new tasks. 

## turtle_client
Client node that acts as the interface between the turtlebot and the planning server. Every turtle client must have a unique clientID in order for the multi-client management to work. ClientIDs can be assigned using the turtle_client_launch.py file and its optional parameter clientID:=XX, where XX is an integer id.

Client receives tasks from the server using the /clientTask topic.

Tasks received from the server are handled in TurtleClient::callback_clientTask() which can be found in TurtleClient_clientTask.cpp.

There are four types of tasks available:
- TASK_PICKUP
- TASK_DROPOFF
- TASK_IDLE
- TASK_CHARGE

TASK_PICKUP - Client moves to pickup landmark, performs camera docking action and picks package. Elevator stays retracted.<br>
TASK_DROPOFF - Client moves to dropoff landmark, performs camera docking action and drops package. Elevator extends to drop package, retracts after a short delay.<br>
TASK_IDLE - Client moves to idle landmark.<br>
TASK_CHAGE - Client moves to charging station and performs create3 docking action.<br>

### Pings the server periodically (/clientState), providing information about:
- Odometry
- Battery
- State (idle/busy/charging/...)

### Talks and interacts with the following turtlebot interfaces:
Topics:
- odometry (/odom)
- docking status (/dock)
- battery (/battery_state)

Services:
- elevator_server (/elevator_service)

Actions:
- charging station docking (/dock, /undock)
- linear movements (/drive_distance)
- nav2 (/navigate_to_pose)
- camera docking (/dock_turtle)

## task_generator
Serves to generate a dummy task as used in the task planner. Was/can be used for debugging purposes.

<p>&copy Hansi, 08.02.2023</p>
