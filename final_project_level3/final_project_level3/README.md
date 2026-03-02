Step 6 - Adapt for Turtlebot3 Simulation in Gazebo

sudo apt install ros-humble-gazebo*
sudo apt install ros-humble-turtlebot3*
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo empty_world.launch.py 