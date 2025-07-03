# 🚗 decision_node – Autonomous Emergency Braking Logic (ROS 2)

This ROS 2 package implements the **Decision Node** for an Autonomous Emergency Braking (AEB) system. It calculates and publishes the required braking intensity in real time based on the vehicle’s current speed and the distance to the nearest detected object. This module is designed to integrate into a multi-node AEB system and is compatible with simulation environments like IPG CarMaker.

---

## 📦 Package Overview

**ROS 2 Distribution:** Humble  
**Platform:** Ubuntu 22.04 (WSL2 supported)  
**Language:** Python (rclpy)

**Folder Structure:**
decision_node/
├── decision_node/
│ ├── init.py
│ └── decision_logic.py ← core decision-making logic
├── setup.py
├── package.xml
└── resource/


---

## 🔧 Node Functionality

The `decision_node` subscribes to real-time input data streams from other vehicle systems and publishes a continuous braking intensity command based on a rule-based decision algorithm.

### 🔄 Subscribed Topics

| Topic Name        | Message Type          | Description                                  |
|-------------------|------------------------|----------------------------------------------|
| `/speed_info`     | `std_msgs/msg/Float32` | Current speed of the vehicle (in m/s)        |
| `/fused_distance` | `std_msgs/msg/Float32` | Estimated distance to the closest obstacle (in meters), fused from radar and depth sensors |

### 📤 Published Topic

| Topic Name       | Message Type          | Description                                         |
|------------------|------------------------|-----------------------------------------------------|
| `/brake_command` | `std_msgs/msg/Float32` | Braking intensity ranging from `0.0` (no braking) to `1.0` (full emergency brake) |

---

## 🧠 Braking Decision Logic

The decision logic is based on distance-to-object and vehicle speed. It computes a float value representing the required braking intensity:

- If `distance >= 20.0`: → `0.0` (no brake)
- If `distance <= 5.0`: → `1.0` (emergency brake)
- For distances in between: → linearly scaled value from `0.0` to `1.0`
- Intensity is further adjusted based on speed:
  - High speed (>30 m/s): scaled up
  - Low speed (<10 m/s): scaled down
- The node executes this logic every 0.1 seconds (10Hz)

---

## 🛠️ Building the Package

Open a terminal and build your ROS 2 workspace:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
🚀 Running the Node
After building, run the node using:

ros2 run decision_node decision_logic
Make sure you’ve sourced your workspace beforehand.

🧪 Manual Testing
Use the following test commands in separate terminals:

# Publish speed
ros2 topic pub /speed_info std_msgs/msg/Float32 '{data: 30.0}'

# Publish object distance
ros2 topic pub /fused_distance std_msgs/msg/Float32 '{data: 6.0}'

# Observe output
ros2 topic echo /brake_command
You should observe a continuous float output representing brake intensity (e.g., 0.6, 0.75, etc.).

🧰 Dependencies
ROS 2 Humble

std_msgs (included with ROS 2 core packages)

📝 Modifying the Logic
To adjust the logic for new braking rules or simulation requirements, edit:

decision_node/decision_node/decision_logic.py
Update the calculate_brake_intensity() method to reflect your new model or equations.


📌 Notes
This module assumes inputs from a properly fused sensor system (/fused_distance)

Braking output is continuous and designed to be fed into an actuator or emergency braking controller

Compatible with future launch file integration and IPG CarMaker scenarios

