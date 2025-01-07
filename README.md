# Robot Navigation with Action Client and Service Nodes

## Short Description of All Components

This project involves a robot navigation system using ROS 2, where:
1. **Action Client Node**: Allows users to set navigation targets, monitor progress, and cancel tasks. It also publishes the robot's position and velocity.
2. **Service Node**: Provides a service to return the last target coordinates.
3. **Launch File**: Starts the simulation environment, action client, and service node.

## How to Run and how to use

### 0. Clone the Repository
Clone the repository into  ROS 2 workspace in the src folder:
```bash
git clone <URL_of_this_repository>
```

### 1.1 Open a terminal and launch the following command:
```bash
roslaunch Simulation_pkg my_launcher.launch
```
### 1.2 Selection of the action for the action_client (in the already launched terminal)
To set a goal:
```bash
set
```
To cancel a selected goal:
```bash
cancel
```

### 1.3 Coordinates choice
Insert the x and the y in a single line, example:
```bash
0 8
```

### 2. Open a new terminal to get the target service

type: 
```bash
rosservice call /get_target
```

### 2. Open a new terminal to get the position velocity msg

type: 
```bash
rostopic echo /position_velocity
```

## Description of action_client_node

The `action_client_node` is responsible for interacting with an action server, sending goals, monitoring their status, and publishing the robot's current position and velocity.

### Key Features:
- **User Interaction**: The node accepts user input to set or cancel goals.
- **Action Client**: Manages goals using ROS 2 action clients, sending targets to the robot and receiving feedback.
- **Odometry Processing**: Subscribes to the `/odom` topic to get the robot's position and velocity.
- **Publishing Position and Velocity**: Publishes this data as a custom message `position_velocity`.

### Code Overview:
- **Action Client Initialization**: The action client is set up to communicate with the `/reaching_goal` action server.
- **Odometry Callback**: The `odomCallback` function processes odometry messages and updates the `position_velocity` topic with the robot's current state.
- **Goal Monitoring Thread**: A separate thread checks the status of the goal, allowing users to know when the robot reaches the target or if the goal is aborted.
- **User Commands**: The node accepts user commands:
  - `set` to set a goal.
  - `cancel` to cancel an ongoing task.

### Relevant Topics:
- **Subscribed**: `/odom` (`nav_msgs/Odometry`)
- **Published**: `position_velocity` (`Simulation_pkg/PositionVelocity`)

### Relevant Actions:
- **Action Server**: `/reaching_goal` (`assignment_2_2024::PlanningAction`)



## target_service_node

The `target_service_node` provides a service to return the last target coordinates sent by the user, ensuring the robot knows its last set goal.

### Key Features:
- **Service Provider**: Offers the `get_target` service, which responds with the last known target coordinates.
- **Target Updates**: Subscribes to `position_velocity` to keep track of the latest goal coordinates sent by the `action_client_node`.

### Code Overview:
- **Service Initialization**: The node advertises the `get_target` service, which clients can call to retrieve the last target coordinates.
- **Target Update Callback**: Listens to `position_velocity` messages to update the stored target coordinates.
- **Thread Safety**: Uses a mutex to ensure thread-safe access to the target coordinates when updating or providing them.

### Relevant Topics:
- **Subscribed**: `position_velocity` (`Simulation_pkg/PositionVelocity`)

### Relevant Services:
- **Service Provided**: `get_target` (`Simulation_pkg::TargetService`)



