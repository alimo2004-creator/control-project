# control-project
made by Ali Mohamed Ahmed Hassan 
junior mechatronics and robotics

# Bicycle Gym — ROS 2 Self-Driving Vehicle Simulation

## 1. Overview & System Architecture

Bicycle Gym is a ROS 2 simulation environment where a vehicle model is controlled around a racetrack. The vehicle is modeled with realistic dynamic constraints: velocity is treated as an integrated state affected by acceleration, drag, and friction. The system is split across three packages:

* **`bicycle_sim`**: Handles vehicle kinematics, physics integration, URDF/Xacro models, and RViz visualizations.
* **`bicycle_control`**: Implements the teleoperation bridge, longitudinal PID cruise control, velocity profiling, and lateral steering controllers (Lateral PID, Pure Pursuit, Kinematic MPC).
* **`track_environment`**: Loads racetrack waypoints (`centerline_0.csv`), publishes path markers, and monitors telemetry via `lap_analyzer`.

---

## 2. Mathematical Formulations

### 2.1 Vehicle Kinematics
The state vector is defined as $x = [x, y, \theta, v]^T$, where $(x,y)$ represents the rear-axle center[cite: 3]. State transitions are updated via Forward Euler integration:

$$x_{k+1} = x_k + v_k \cos(\theta_k) \Delta t$$
$$y_{k+1} = y_k + v_k \sin(\theta_k) \Delta t$$
$$\theta_{k+1} = \theta_k + \frac{v_k}{L} \tan(\delta_k) \Delta t$$
$$v_{k+1} = v_k + a_k \Delta t$$

where $L = 1.25\text{ m}$ is the wheelbase, and $\delta_k$ is the steering angle[cite: 3]. Heading is normalized within $\theta \in [-\pi, \pi]$, and forward velocity is clamped to $v \ge 0$[cite: 3].

### 2.2 Control Laws
* **Longitudinal PID:** Regulates throttle $u_v \in [-1, 1]$ to maintain a target speed $v_{\text{target}}$, using anti-windup clamping to prevent integrator saturation[cite: 3].
* **Lateral PID:** Evaluates cross-track error ($e_y$) and heading error ($e_\theta$) to command front steering angle $\delta = -K_p e_y - K_d \dot{e}_y - K_\theta e_\theta$[cite: 3].
* **Pure Pursuit:** Calculates curvature $\kappa = \frac{2 \sin(\alpha)}{L_d}$ toward a look-ahead point at distance $L_d(v) = k_p v + L_0$[cite: 3].
* **Kinematic MPC:** Solves a constrained optimization problem over prediction horizon $N$ in the Frenet frame to minimize tracking errors while accounting for actuator physical constraints ($\delta \in [\delta_{\min}, \delta_{\max}]$)[cite: 3].

---

## 3. Benchmark Comparison Table

The telemetry metrics below were gathered by `lap_analyzer` across 3 full laps for each control mode[cite: 1, 2, 3, 4]:

| Controller Mode | Best Lap Time (s) | Top Speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Laps Evaluated |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Manual Teleop (Cruise)** | 103.816 | 5.26 | 1.082 | 5.097 | 1.384 | 3 |
| **Lateral PID** | 84.147 | 7.32 | 0.408 | 2.733 | 0.594 | 3 |
| **Pure Pursuit** | 75.219 | 7.30 | 0.064 | 0.339 | 0.088 | 3 |
| **Kinematic MPC** | 123.822 | 4.13 | 0.071 | 0.348 | 0.097 | 3 |

---

## 4. Theoretical Analysis: MPC vs. Pure Pursuit & Lateral PID

1. **Reactive vs. Predictive Control:**
   * **Lateral PID** operates strictly on instantaneous error. Due to powertrain lag and steering limits, this causes overshoot and oscillation at higher speeds (max CTE of 2.733 m).
   * **Pure Pursuit** introduces geometric preview ($L_d$), reducing path deviation significantly (RMS CTE down to 0.088 m)[cite: 3]. However, it lacks a multi-step dynamic model, making it susceptible to cutting corners at high velocities.
   * **Model Predictive Control (MPC)** formulates path tracking as a constrained receding horizon optimization problem. By evaluating vehicle model dynamics over $N$ steps ahead, it anticipates upcoming track curvature and optimizes steering commands smoothly within hardware limits.

2. **Speed-Profile Constraints:**
   * In this setup, MPC operated with lower peak velocity limits (Top Speed $4.13\text{ m/s}$), yielding consistent path tracking with virtually zero overshoot (RMS CTE $0.097\text{ m}$). Pure Pursuit achieved higher speed ($7.30\text{ m/s}$) due to its fixed look-ahead distance law.

---

## 5. Milestone 6: Free Exploration Summary

## 1. Four-Wheel Ackermann Kinematics & ros2_control

## 2. Modern 3D Simulation Environments (Gazebo & MVSim)

## 3. Stochastic Sampling-Based Predictive Control (Nav2 MPPI)

---

## 6. Reproduction Guide
```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Install build, simulation, and controller dependencies
sudo apt update && sudo apt install -y python3-colcon-common-extensions \
  python3-numpy python3-scipy ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-xacro
```
In every new terminal, source the ROS distribution and built workspace:
```bash
source /opt/ros/humble/setup.bash
cd /path/to/bicycle_gym-main
source install/setup.bash
```
lunch modes and building
```bash
# making the directory
mkdir /mnt/g/Control_Project-mainARL/bicycle_gym-main

# Build the workspace (bicycle_sim, bicycle_control, track_environment)
cd /Control_Project-mainARL/bicycle_gym-main
colcon build --symlink-install
source install/setup.bash

# Launch Simulation & Control
ros2 launch bicycle_sim bicycle_sim.launch.py
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=teleop
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=lateral_pid
ros2 launch bicycle_control bicycle_gym.launch.py controller_mode:=pure_pursuit
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=mpc

# Launch Lap Analyzer separately (if not auto-started)
ros2 run track_environment lap_analyzer
```
# Milestone 6: Free Exploration & Reference Resources


