# control-project
made by Ali Mohamed Ahmed Hassan 
junior mechatronics and robotics

# ROS 2 Self-Driving Vehicle Simulation
* Note: a large portion of the codes were made by vibe coding as i am still learning python and not a sith lord in it 😄😄
## video demonistrating the whole task
(https://drive.google.com/file/d/1zSk6OEKvWxsOTGHKoJj-xNTX1sAuIc6x/view?usp=sharing)

## 1. Overview & System Architecture

Bicycle Gym is a ROS 2 simulation environment where a vehicle model is controlled around a racetrack. The vehicle is modeled with realistic dynamic constraints: velocity is treated as an integrated state affected by acceleration, drag, and friction. The system is split across three packages:

* **`bicycle_sim`**: Handles vehicle kinematics, physics integration, URDF/Xacro models, and RViz visualizations.
* **`bicycle_control`**: Implements the teleoperation bridge, longitudinal PID cruise control, velocity profiling, and lateral steering controllers (Lateral PID, Pure Pursuit, Kinematic MPC).
* **`track_environment`**: Loads racetrack waypoints (`centerline_0.csv`), publishes path markers, and monitors telemetry via `lap_analyzer`.

---

## 2. Mathematical Formulations

### 2.1 Vehicle Kinematics
The state vector is defined as $x = [x, y, \theta, v]^T$, where $(x,y)$ represents the rear-axle center. State transitions are updated via Forward Euler integration:

$$x_{k+1} = x_k + v_k \cos(\theta_k) \Delta t$$
$$y_{k+1} = y_k + v_k \sin(\theta_k) \Delta t$$
$$\theta_{k+1} = \theta_k + \frac{v_k}{L} \tan(\delta_k) \Delta t$$
$$v_{k+1} = v_k + a_k \Delta t$$

where $L = 1.25\text{ m}$ is the wheelbase, and $\delta_k$ is the steering angle. Heading is normalized within $\theta \in [-\pi, \pi]$, and forward velocity is clamped to $v \ge 0$.

### 2.2 Control Laws
* **Longitudinal PID:** Regulates throttle $u_v \in [-1, 1]$ to maintain a target speed $v_{\text{target}}$, using anti-windup clamping to prevent integrator saturation.
* **Lateral PID:** Evaluates cross-track error ($e_y$) and heading error ($e_\theta$) to command front steering angle 
$delta$ = $-K_p$  $e_y$ - $K_d$ $\dot{e}_y$ - ($K_i$ \ $theta$) ($e_y$ \ $theta$).
* **Pure Pursuit:** Calculates curvature $\kappa = \frac{2 \sin(\alpha)}{L_d}$ toward a look-ahead point at distance $L_d(v) = k_p v + L_0$.
* **Kinematic MPC:** Solves a constrained optimization problem over prediction horizon $N$ in the Frenet frame to minimize tracking errors while accounting for actuator physical constraints ($\delta \in [\delta_{\min}, \delta_{\max}]$).

---

## 3. Benchmark Comparison Table

The telemetry metrics below were gathered by `lap_analyzer` across 3 full laps for each control mode:

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
   * **Pure Pursuit** introduces geometric preview ($L_d$), reducing path deviation significantly (RMS CTE down to 0.088 m). However, it lacks a multi-step dynamic model, making it susceptible to cutting corners at high velocities.
   * **Model Predictive Control (MPC)** formulates path tracking as a constrained receding horizon optimization problem. By evaluating vehicle model dynamics over $N$ steps ahead, it anticipates upcoming track curvature and optimizes steering commands smoothly within hardware limits.

2. **Speed-Profile Constraints:**
   * In this setup, MPC operated with lower peak velocity limits (Top Speed $4.13\text{ m/s}$), yielding consistent path tracking with virtually zero overshoot (RMS CTE $0.097\text{ m}$). Pure Pursuit achieved higher speed ($7.30\text{ m/s}$) due to its fixed look-ahead distance law.

---

## 5. Reproduction Guide

unzip the files where they contain the codes and all related packages, also contains data folder that has images and data values for each controller

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
## Milestone 6: Free Exploration & Reference Resources

## 1. Four-Wheel Ackermann Kinematics & ros2_control

Four-wheel Ackermann kinematics describes how the steering angles of a vehicle's front wheels must differ when turning. The inside wheel follows a smaller turning radius than the outside wheel, so it must turn at a sharper angle. If both wheels use the same steering angle, tire scrubbing, increased wear, and energy losses can occur.

The steering angles are calculated using the wheelbase $L$, track width $W$, and turning radius $R$:

$$\tan(\delta_{\text{inner}}) = \frac{L}{R - W/2}$$

$$\tan(\delta_{\text{outer}}) = \frac{L}{R + W/2}$$

ROS 2 provides tools for implementing these steering relationships through `ros2_control`, a framework for connecting controllers to robot hardware interfaces. Its steering controller library supports Ackermann and bicycle steering configurations, while its documentation and demonstrations provide examples of wheel kinematics, hardware integration, and controller implementation.

These resources help developers move from simplified bicycle models toward more realistic four-wheel vehicle models. They are useful for autonomous cars and automated mobile robots (AMRs), where accurate steering coordination is essential for reliable movement.

### Key Resources
- ROS 2 Control Mobile Robot Kinematics
- ROS 2 Steering Controllers Library
- Steered Wheel Base Demo
- Four-Wheel AMR Implementation

---

## 2. Modern 3D Simulation Environments (Gazebo & MVSim)
# Modern 3D Simulation Environments for Autonomous Vehicles: Gazebo and MVSim

## 1. Introduction

Modern 3D simulation environments, such as Gazebo and MVSim, are important tools for developing and testing autonomous vehicles. Unlike basic 2D simulations, which mainly focus on vehicle movement and steering geometry, 3D simulators can represent more realistic physical interactions between the vehicle, its tires, and the surrounding environment.

In a simple kinematic bicycle model, the vehicle is usually assumed to move without tire slipping. This assumption is useful for designing steering controllers, but it does not fully represent the physical behavior of a real vehicle. In reality, tire friction, suspension movement, road conditions, and vehicle dynamics affect how the vehicle responds to steering and acceleration commands.

Modern simulation environments help bridge this gap by providing physics-based vehicle models, simulated sensors, and configurable environments. They allow engineers to test autonomous driving algorithms before implementing them on real vehicles.

## 2. Why Use 3D Vehicle Simulation?

A kinematic model is computationally efficient and suitable for developing controllers such as Pure Pursuit, PID, and Model Predictive Control (MPC). However, it cannot fully reproduce the physical behavior of a real vehicle.

Three-dimensional physics simulation can represent several important factors:

- **Tire friction and slipping:** Tires may lose grip during sharp turns or rapid acceleration and braking.
- **Suspension dynamics:** The vehicle body can respond to uneven terrain and changes in wheel loading.
- **Tire-road interaction:** Contact between tires and the road affects steering, braking, and stability.
- **Sensor simulation:** Simulated sensors can provide measurements with configurable noise and other imperfections.
- **Terrain effects:** Vehicles can be tested on different surfaces and road geometries.

Some advanced vehicle models use the Pacejka Magic Formula to approximate tire forces under different slip conditions. However, not every simulator includes this tire model by default; the available physics depend on the simulator, vehicle model, and configuration.

These capabilities make 3D simulation particularly useful for evaluating whether a controller remains stable under conditions that a simple kinematic model cannot represent.

## 3. Curated Simulation Platforms

### 3.1 Modern Gazebo (Gz-Sim) with ROS 2

**Resource:** [Ackermann Vehicle in Modern Gazebo & ROS 2](https://github.com/alitekes1/ackermann-vehicle-gzsim-ros2)

Modern Gazebo, also known as Gazebo Sim or Gz-Sim, is a robotics simulation environment that supports physics-based simulation and integration with ROS 2.

The referenced project demonstrates an autonomous Ackermann-steering vehicle using modern Gazebo and ROS 2. Ackermann steering is commonly used in cars, where the front wheels turn at different angles to follow a curved path correctly.

Gazebo is useful for connecting a vehicle model to a complete autonomous driving system. ROS 2 nodes can process vehicle states, generate reference paths, calculate steering and throttle commands, and communicate with the simulated vehicle.

**Main applications:**

- Testing steering and trajectory-tracking algorithms.
- Integrating ROS 2 controllers with simulated vehicles.
- Evaluating vehicle behavior in different simulated environments.
- Developing and debugging autonomous driving systems before physical testing.

For a project using Pure Pursuit, Gazebo can help evaluate how steering performance changes with vehicle speed, road curvature, and simulated physical effects.

### 3.2 Classic Gazebo Ackermann Simulation

**Resource:** [Classic Gazebo Ackermann Steering Vehicle](https://github.com/lucasmazzetto/gazebo_ackermann_steering_vehicle)

Gazebo Classic is an older generation of the Gazebo simulation platform. The referenced project focuses on simulating a vehicle with Ackermann steering, including the physical steering mechanism.

The main benefit of this type of simulation is the ability to study the relationship between steering commands and the movement of the vehicle's wheels. Rather than considering steering only as a mathematical angle, the model can represent steering components and their mechanical behavior.

This makes the project relevant for understanding how a steering mechanism operates and how its geometry affects vehicle movement.

**Main applications:**

- Studying Ackermann steering geometry.
- Understanding the relationship between steering inputs and wheel angles.
- Testing vehicle models with mechanical steering components.
- Learning how simulated vehicle mechanisms interact with physics.

One limitation is that Gazebo Classic belongs to an older simulation generation. When starting a new ROS 2 project, engineers should check compatibility with their ROS 2 distribution and consider whether modern Gazebo would be more appropriate.

### 3.3 Ackermann Autonomous Car Simulation

**Resource:** [Ackermann Autonomous Car Simulation](https://github.com/armando-genis/Ackermann-Autonomous-Car-Simulation)

This project provides an autonomous car simulation built around Ackermann vehicle kinematics and an autonomous driving stack.

Its relevance lies in the relationship between vehicle modeling and autonomous control. A vehicle controller must use information about the vehicle's position, orientation, and speed to determine appropriate steering and acceleration commands.

A simulation environment allows these algorithms to be tested repeatedly without requiring a physical vehicle. Engineers can evaluate whether the vehicle follows a reference path, investigate tracking errors, and adjust controller parameters.

**Main applications:**

- Developing autonomous driving algorithms.
- Testing reference-path tracking.
- Evaluating steering control strategies.
- Studying the integration of vehicle models and autonomous navigation.

This type of project is particularly relevant when developing a controller that must follow paths while respecting the steering limitations of an Ackermann vehicle.

### 3.4 MVSim — Multi-Vehicle Simulator for ROS 2 Humble

**Resource:** [Official ROS 2 Humble MVSim Tutorial](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/MVSim/Simulation-MVSim.html)

MVSim is a lightweight simulator designed for mobile robots and multi-vehicle applications. The referenced documentation describes its use with ROS 2 Humble.

Compared with a detailed, computationally demanding 3D vehicle simulation, a lightweight simulator can make it easier to run experiments quickly and test algorithms repeatedly.

MVSim is useful when the primary objective is to evaluate vehicle behavior and navigation rather than reproduce every mechanical detail of a full-size vehicle.

**Main applications:**

- Testing mobile robot navigation and control.
- Running multi-vehicle simulation scenarios.
- Developing and testing ROS 2 applications.
- Performing repeated experiments with relatively low simulation overhead.

Its suitability for detailed tire-slip or suspension studies depends on the available vehicle models and physics features. Therefore, it should not automatically be considered equivalent to a high-fidelity vehicle dynamics simulator.

## 4. Comparison of the Simulation Environments

| Platform | Main focus | Main advantage | Important consideration |
|---|---|---|---|
| Modern Gazebo | Physics-based robotics and vehicle simulation | ROS 2 integration and configurable environments | Setup and model complexity |
| Gazebo Classic | Vehicle mechanisms and physical simulation | Useful for studying steering geometry | Older platform with compatibility considerations |
| Ackermann Autonomous Car Simulation | Autonomous driving and path tracking | Focus on vehicle control algorithms | Capabilities depend on the project implementation |
| MVSim | Lightweight mobile robot and multi-vehicle simulation | Fast experimentation | Detailed tire and suspension behavior must be verified |

## 5. How These Simulators Support Controller Development

A practical development process begins with a simple kinematic model. A controller such as Pure Pursuit can be developed and tested in this environment because it calculates steering commands from the vehicle's position, heading, and a target point on the reference path.

After the controller works reliably, it can be integrated into a more physically representative simulation.

The process generally involves:

1. **Develop the controller:** Implement the steering algorithm and verify its mathematical behavior.
2. **Integrate with ROS 2:** Connect the controller to vehicle-state and reference-path topics.
3. **Test vehicle tracking:** Evaluate how accurately the vehicle follows different paths.
4. **Introduce physical effects:** Where supported, enable tire friction, slipping, suspension behavior, and sensor noise.
5. **Evaluate robustness:** Observe how the controller behaves when the vehicle encounters more demanding driving conditions.

For example, a Pure Pursuit controller might track a curved path successfully in a kinematic simulation but behave differently when tire grip becomes limited. Testing in a physics-based environment can reveal these limitations and help engineers determine whether the controller needs additional constraints or a more advanced vehicle model.

## 6. Conclusion

Modern Gazebo, Gazebo Classic, Ackermann autonomous vehicle projects, and MVSim offer different approaches to vehicle simulation. Gazebo is particularly useful for physics-based simulation and ROS 2 integration, while MVSim emphasizes lightweight and efficient experimentation. The Ackermann-focused projects provide practical examples of steering mechanisms and autonomous vehicle control.

The best choice depends on the objective. For rapid controller development, a lightweight or kinematic simulator may be sufficient. For evaluating realistic vehicle behavior, including tire friction and suspension effects, a suitable physics-based simulation is more appropriate. Combining both approaches provides a useful progression from controller design to more realistic autonomous vehicle testing.

--- 

## 3. Stochastic Sampling-Based Predictive Control (Nav2 MPPI)
Model Predictive Path Integral (MPPI) control is an advanced control method designed to find effective control actions for complex, nonlinear systems. Instead of evaluating only one possible future movement, MPPI samples thousands of candidate trajectories in parallel and evaluates their performance using a cost function.

Each trajectory receives a cost based on factors such as path-following accuracy, control effort, and obstacle avoidance. The algorithm uses path-integral weighting to combine the sampled control sequences and determine an improved control action. This process repeats as the vehicle moves, allowing the controller to adapt to changing conditions.

The Nav2 MPPI Controller provides an implementation within the ROS 2 Navigation stack. It supports customizable cost functions and dynamic obstacle avoidance, making it useful for autonomous navigation in environments where the robot must continuously adjust its trajectory.

MPPI is particularly valuable when conventional controllers struggle with complex constraints or nonlinear vehicle behavior. Its performance depends on factors such as the number of sampled trajectories, computational resources, motion model, and cost-function design. Parallel processing can improve efficiency, although GPU acceleration is not required in every implementation.

### Key Resource
- Nav2 MPPI Controller Documentation

