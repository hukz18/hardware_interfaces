# hardware_interface
This repo is adapted from Yifan Hou's [hardware_interface](https://github.com/yifan-hou/hardware_interfaces) repo to provide addmitance force control for the robot-trains-robot arm teacher. Pleasea refer refer to the original repo for installation information.
The cpp controller communicates with the python arm leader to provide necessary information and adjust accordingly.

Currently supporting the following interfaces:
* Position-controlled robot arm
* 6-dof force torque sensor

Current implementations includes (see `robots/`):
* UR robot rtde communication
* ARX robot arm via CAN bus
* ATI force torque sensor via netft;
* Robotiq FT series force torque sensor via modbus
* Realsense cameras via USB.

# Install
Please refer to the [hardware_interface](https://github.com/yifan-hou/hardware_interfaces) repo for installation guide.