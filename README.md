# TSWR controllers

Repository contains 2 controllers:
- [x] Cross Entropy Method controller (CEM)
- [x] Improved Cross Entropy Method controller (iCEM)

## Requirements
First thing required is having f1tenth simulator running along with ros humble. Second requirement is to create new ros workspace and clone this repository to src directory as well as autoware_auto_msgs with commands:
```bash
git clone https://github.com/conegiorno/tswr_controllers.git
```
afterwards
```bash
git clone https://github.com/tier4/autoware_auto_msgs
```
After that build autoware messages with command:
```bash
colcon build --packages-select autoware_auto_control_msgs
```
After wards source to with:
```bash
source install/setup.bash
```
Remove build/ and log/ folders and then build entire workspace with:
```bash
colcon build --symlink-install
```

# Running controllers with simulators
There is an option of running any controller along with simulator right of the bat. The only thing needed is to have simulator installed. When it is installed and running in background using controllers is conducted with simple ros commands.

## Cross Entropy Method controller
Running cross entropy method is conducted via following command:
```bash
ros2 launch tswr_controllers cem_controller_launch.py 
```

## Stanley controller
Running improved cross entropy method is conducted via following command:
```bash
ros2 launch tswr_controllers icem_controller_launch.py
```

