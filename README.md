# PIASR Group 12
This repository contains the drivers, visualization features and a simple simulation for [LeRobot] https://github.com/huggingface/lerobot) with the addition of Group 12's code for the PIASR assignment.

## Usage
Most of the code runs on ROS2, with the exception of the Workspace directory visible at the highest level. All other code is contained under the python_controllers package.

To run the code, go to the workspace and build the ros packages with the following terminal commands:

        git submodule update --recursive
        
        source /opt/ros/{ROS_DISTRO}/setup.bash

        colcon build

        source install/setup.bash

Then run one of the following launch files:

**Launch files (lerobot package)**

| Command | Effect |
|---------|--------|
| `ros2 launch lerobot sim_position.launch.py` | Simulation and RViz in position control mode |
| `ros2 launch lerobot sim_velocity.launch.py` | Simulation and RViz in velocity control mode |
| `ros2 launch lerobot rviz.launch.py` | RViz only. **Launch something else** (sim, hw, or joint_slider) in another terminal to see the robot at a real pose. |
| `ros2 launch lerobot joint_slider.launch.py` | Joint position slider GUI (no RViz) |
| `ros2 launch lerobot hw_position.launch.py` | Hardware interface in position control mode |
| `ros2 launch lerobot hw_velocity.launch.py` | Hardware interface in velocity control mode |
| `ros2 launch lerobot hw_read.launch.py` | Passive read-only: publish joint states, torque disabled (move robot by hand) |
        
As all our code is contained in the python controllers package type:

        ros2 run python_controllers 

Add a space to the end of the command and press tab to see a list of files. The file names give a rough description of what they do. Then use

        ros2 run python_controllers competition

To run the competition.py file for example.
