# Street Line Painting Robot

This project controls the **mp400** robot for street line painting using ROS and MQTT.

## Setup Instructions

1. **Clone the Repository**  
   ```bash
   git clone https://github.com/islamalama22/Street_robot.git
   ```

2. **Install mp400 Packages**  
   Follow instructions from [nebotix/mp400/simulation](https://github.com/neobotix/mp400_simulation).

3. **Create and Build ROS Workspace**  
   ```bash
   mkdir -p ~/mp400_ws/src
   cd ~/mp400_ws/src
   catkin_init_workspace
   cd ..
   catkin_make
   source devel/setup.bash
   ```

4. **Install Dependencies**  
   ```bash
   sudo apt update
   sudo apt upgrade
   sudo apt install python3-pip
   pip3 install paho-mqtt
   ```

5. **Create a Map**  
   Use ROS commands to map the environment for robot navigation.

6. **Run the System**  
   Launch ROS nodes to start the robot's movement and painting operations.

## Technologies Used
- **ROS Noetic** (Ubuntu 20.04)
- **MQTT** for data communication
- **Python 3** for scripting

## Contact
For questions, contact **islamalama22@gmail.com**.

---

Enjoy painting with your robot!
