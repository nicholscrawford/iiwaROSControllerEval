# iiwaROSControllerEval
Python ROS tools for evaluating the command tracking of controllers.

## Position Tracking

Run ```python joint_error_plotter.py``` to plot the joint errors. The system expects joint commands on the ros topic ```/iiwa/joint_cmd``` and joint states on ```/iiwa/joint_states``` where both are ```JointState ``` messages. The typical total error on the current system is something like 1.2-1.8

## Force Tracking

TODO: Implement applied force monitoring.

## System Energy Monitoring

TODO: Implement system energy monitoring for collision detection as described in [Collision Detection and Safe Reaction with the DLR-III Lightweight Manipulator Arm](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4058607&tag=1) using dynamic parameters obtained from [Dynamic Identification of the KUKA LBR iiwa](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9112185). Need published joint velocities and torques.
