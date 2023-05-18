# Robust Trajectory Tracking Control for Quadrotor UAVs	

Team : **[Ankush Singh Bhardwaj](https://github.com/ankushsingh999)**   &  **[Anuj Pai Raikar](https://github.com/22by7-raikar)**

***Complying to the code of conduct laid out by the professor we have made the repo containing codes and derivation of the control laws private.***

This project aims to develop a robust control scheme to enable a quadrotor, specifically the Crazyflie 2.0 platform, to track desired trajectories in the presence of external disturbances. The control design will be implemented and tested using MATLAB and ROS (Robot Operating System).

![image](https://github.com/ankushsingh999/Robust-Traj-Control-for-UAVs/assets/64325043/1720dcde-6ee9-432f-9ca6-9e6e3d220441)


# Crazyflie 2.0 Overview
The Crazyflie 2.0 is a micro air vehicle (MAV) quadrotor weighing only 27 grams. It is equipped with four 7mm coreless DC motors, providing a maximum takeoff weight of 42g. The quadrotor's physical parameters are listed below:

![image](https://github.com/ankushsingh999/Robust-Traj-Control-for-UAVs/assets/64325043/b071ffd7-3ff3-4097-9170-b6ba69259702)

# Dynamic Model and Control Design
The quadrotor dynamic model is described by the equations of motion, considering the translational and angular coordinates. The control inputs on the system are the force and moments applied by the propellers. A sliding mode control approach is used to track desired trajectories for altitude and attitude control.

The project consists of the following parts:

**Trajectory Generation:** MATLAB or Python script to generate quintic trajectories for the translational coordinates (x, y, z) of the Crazyflie quadrotor. The quadrotor will visit five waypoints sequentially, and the desired trajectories will be plotted. The waypoints to visited are:

![image](https://github.com/ankushsingh999/Robust-Traj-Control-for-UAVs/assets/64325043/4bd9b204-6689-46b8-8a19-70bd01643f24)

The sequence of visiting the waypoints does matter. The velocity and acceleration at each waypoint must be equal to zero.

**Sliding Mode Controller:** Designed and implemented boundary layer-based sliding mode control laws for altitude and attitude control. The control laws will enable the quadrotor to track the desired trajectories and visit the waypoints.

To convert the desired position trajectories (xd, yd, zd) to desired roll and pitch angles (ϕd, θd), the desired forces in x and y direction can be calculated using PD control (according to Eq. (1) and (2)), and the resulting desired forces can be then converted to desired ϕ and θ according to Eq. (3) and Eq. (4):

![image](https://github.com/ankushsingh999/Robust-Traj-Control-for-UAVs/assets/64325043/f66f9a2a-cee6-40b8-9cf5-df39ff71a0c5)

![image](https://github.com/ankushsingh999/Robust-Traj-Control-for-UAVs/assets/64325043/7a5b0645-4a50-4dde-8650-f07f08e00ac5)



**ROS Implementation:** Developed a ROS node in Python or MATLAB to evaluate the performance of the control design on the Crazyflie 2.0 quadrotor in Gazebo. The ROS node incorporated the trajectory generation and sliding mode control laws.


# RESULTS

Tested the control script developed and observed the quadrotor to move smoothly with minimal overshoot or oscillations to track the trajectories generated.

https://github.com/ankushsingh999/Robust-Traj-Control-for-UAVs/assets/64325043/461eecf5-ae48-4f56-ae06-22efd8e68120

A visualization script was run to plot actual trajectories on top of the reference trajectories to ensure satisfactory performance was achieved.

![trajectory](https://github.com/ankushsingh999/Robust-Traj-Control-for-UAVs/assets/64325043/78abeaa3-08bf-4b26-9b88-b6572cb871b9)

The video and the graph show us that the UAV converges to the reference trajectory in fraction of seconds and the error between the desired and actual trajectories is minimal. 

