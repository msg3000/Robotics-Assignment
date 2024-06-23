# Surveillance Bot

An autonomous agent that is able to navigate and monitor a particular environment

## Running

### World Environment

In order to download the world environment, the following commands should be run from a ros terminal, with capacity to run ros-kinetic.
```bash
wget https://lamp.ms.wits.ac.za/robotics/robot_assignment_ws.tar.gz
tar zxvf robot_assignment_ws.tar.gz
cd robot_assignment_ws
```

This repository can be cloned at the root of this workspace.

In order to compile the scripts and build:
```bash
rm -r build
catkin_make
```

The world environment can now be launched through running:
```bash
./startWorld
```

### Surveillance Bot

Code for the surveillance bot is contained in the `scripts` folder of the `surveillance_bot` package. The code files are structured as follows:

.
    ├── main.py                   # Main script for the surveillance bot
    ├── rrt.py                    # Implements RRT-star algorithm
    ├── navigator.py              # Handles state information and velocity publishing
    ├── mapping.py                # Handles coordinate transformations from world to pixel
    └── pid.py                    # Implements general PID Controller

The main surveillance bot can be run in a new terminal, from the same directory as above using the commands:
```bash
source devel/setup.bash
rosrun surveillance_bot main.py
```

Following this, you will be prompted to enter in the desired coordinate locations of the world.

