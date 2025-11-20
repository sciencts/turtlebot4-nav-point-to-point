# Turtlebo4 Navigation from Point A to Point B

This Project was build for my midterm exam, the purpose is move to 
point A, and active the buzzer one time and go to another point B and active the Buzzer twice.

---
# How To Build
### Create Folder Workspace
```
mkdir -p turtlebot4_ws/src
cd turtlebot4_ws/src
```
### Clone this repo 
```
git clone https://github.com/sciencts/turtlebot4-nav-point-to-point.git
```
### Install package and dependiencies
```
cd ../ && rosdep install --from-paths src --ignore-src -r -y
```
### Build the package
```
colcon build
```
---
# How To Connect from PC to Turtle
### Via Ethernet
```
ssh ubuntu@192.168.185.3 
```
### Via WiFi
```
ssh ubuntu@your_robot_ip 
```
---
# How To use Mapping Mode
### Mapping Launch
```
ros2 launch turtlebot4_navigation slam.launch.py
```
### Rviz Run 
```
ros2 launch turtlebot4_viz view_navigation.launch.py  # This Jazzy

ros2 launch turtlebot4_viz view_robot.launch.py # This Humble
```
### Control Robot Via Teleop Keyboard
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

---
# How To Run Nav2 
### Localization Launch
```
source install/setup.bash
ros2 launch turtlebot4_pick_place localization.launch.py
```
### Navigation Launch
```
source install/setup.bash
ros2 launch turtlebot4_pick_place uts_nav.launch.py
```
### Rviz Run 
```
ros2 launch turtlebot4_viz view_navigation.launch.py  # This Jazzy

ros2 launch turtlebot4_viz view_robot.launch.py # This Humble
```
### Run the send goal point A and B 
```
source install/setup.bash
ros2 run turtlebot4_pick_place turtlebot4_pick_place_node
```

---

# Demo Video

<p align="center">
  <a href="https://www.youtube.com/watch?v=aJAJW27PNXo">
    <img src="https://img.youtube.com/vi/aJAJW27PNXo/hqdefault.jpg" alt="TurtleBot4 Navigation Demo">
  </a>
</p>
