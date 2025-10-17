# Mars Rover Architecture with ROS

## Project Overview

This repository contains the architecture, nodes, and core logic for a Mars rover built using ROS (Robot Operating System).  
The code focuses on building modular ROS nodes (e.g. vision, navigation, coordination) that simulate or control rover behavior.

### Key Goals

- Design a clean node-level architecture (coordination, sensing, action)  
- Facilitate reusability and extension  
- Allow simulation and real deployment with minimal changes  
- Demonstrate ROS best practices (topics, services, parameters)

---


- **urc_missions_interfaces**: Custom ROS action definitions  
- **urc_missions**: Main nodes and mission logic  
- **launch**: Launch files to start nodes  
- **resource**: Static resource files, config, etc.  
- **Support files**: `package.xml`, `setup.py` for ROS package setup  

---

## How It Works (High-Level Flow)

1. **Coordinator Node**  
   - Acts as central controller  
   - Receives goals or mission commands  
   - Delegates tasks among other nodes  

2. **Autonomous Navigation (autonav_node.py)**  
   - Handles path planning, obstacle avoidance  
   - Publishes velocity commands  

3. **Vision Node (vision_node.py)**  
   - Processes camera input  
   - Detects features, returns coordinates  

4. **Science Node (science_node.py)**  
   - Executes mission-specific tasks (sample collection, analysis)  

5. **Delivery / Action Client (delivery_action_client.py)**  
   - Interacts with ROS Action servers (delivery tasks)  

---

## Setup Instructions

1. Clone the repo:

    ```bash
    git clone https://github.com/Showvik2405/Mars-rover-architecture-with-ROS.git
    cd Mars-rover-architecture-with-ROS/urc_ws
    ```

2. Install dependencies (ROS version, Python libs, etc.)

3. Build the workspace:

    ```bash
    colcon build
    source install/setup.bash
    ```

4. Launch the system:

    ```bash
    ros2 launch urc_missions urc.launch.py
    ```

---

## Usage & Commands

- To run only the coordinator node:
  ```bash
  ros2 run urc_missions coordinator_node

To call a delivery action:
```bash
ros2 action send_goal /deliver urc_missions_interfaces/action/DeliverMission "{...}"


Useful ROS2 tools:

ros2 topic echo /topic_name

ros2 service call ...
 ```

---


Example Scenario

1. Coordinator receives a mission like "deliver payload to waypoint A."

2. It forwards to Autonav for navigation path planning.

3. Vision module scans for obstacles or markers.

4. Science node may perform analysis or sampling at the goal.

5. Delivery action client coordinates success/failure reports.

License

This project is licensed under the MIT License. See LICENSE for details.

Contact

Created by Showvik
GitHub: Showvik2405

Feel free to open issues or make pull requests.



---




