# 7DOF-Cobot-FT-MPC-ArticulatingArm-MuJoCo-Python

A 7-DoF collaborative robot arm controller that uses force–torque sensing and model-predictive control to articulate doors and other hinged objects in Mujoco.

---
## Project Overview

This repository provides a reusable Python implementation of a door-opening controller for collaborative robot arms in Robosuite environment of Mujoco. 

Robotsuite: (https://github.com/StanfordVL/robosuite)

It seamlessly blends:

- **Servo-based positioning** toward a handle  
- **6-axis force–torque feedback** for trustworthy contact detection  
- **Model-Predictive Control (MPC)** to plan and execute pull motions that minimize pose, hinge-angle, and wrench costs  
- **Adaptive gripping** to switch grip strength on contact and maintain a secure grasp  

The result is a robust “articulating arm” capable of opening doors, cabinets, valves—and any hinged object—across multiple robot platforms.

---

## Why robosuite?

- **Multi-cobot support**: out-of-the-box simulation of Jaco, Kuka IIWA, Sawyer, Franka Panda, UR5e, and more.  
- **MuJoCo physics**: realistic contact dynamics and friction modeling.  
- **Built-in sensors**: wrist-mounted force–torque, RGB-D cameras, joint encoders.  
- **Modular API**: easy configuration of arms, grippers, environments, and control loops.  
- **Active community**: continuous improvements, example tasks, and research extensions.

By leveraging robosuite, we avoid building low-level simulators ourselves and focus on high-level control logic.

---

## Supported Manipulator Cobots

This controller has been tested on—and can be deployed to—any of robosuite’s standard manipulators:

| Robot         | DOF | Gripper           |
| ------------- | --- | ----------------- |
| Jaco          | 6   | Jaco 3-finger     |
| Kuka IIWA     | 7   | Robotiq 2-finger  |
| Sawyer        | 7   | Two-finger        |
| Franka Panda  | 7   | Panda gripper     |
| UR5e          | 6   | Robotiq 2-finger  |
| Custom arms   | 6–7 | Any robosuite-supported gripper |

Because the code references generic API calls (`env.reset()`, `env.step(action)`, `env.sim.data.sensordata`), swapping among these cobots requires only changing the `make()` arguments in your config.

---

## Key Concepts

### Force–Torque Sensing

- **6-axis wrist sensor** measures forces (Fx, Fy, Fz) and torques (τx, τy, τz).  
- **Soft thresholds** trigger “first contact” when exceeded.  
- **Hard thresholds** penalize excessive wrench in the MPC cost to avoid damage or slipping.  
- **Adaptive feedback**: once in‐grasp, maintain high grip force; if sensor data drops below release threshold, open gripper.

### Model-Predictive Control (MPC)

- **Sampling-based**: at each timestep, sample N candidate action sequences.  
- **Cost function** includes:  
  - Pose error to desired handle frame  
  - Hinge-angle reward (encourage opening)  
  - Wrench penalty (force/torque soft & hard)  
- **Execution**: pick the sequence with lowest cumulative cost and execute its first action.

### Servo-Based Approach

- Drives end-effector quickly to within an **approach distance** of the handle using a PD controller.  
- Closes gripper lightly during approach.

### Adaptive Gripping Logic

1. **Approach & light-close** until `force > f_soft`.  
2. **Full-grip** close when contact is detected.  
3. **Maintain** grip during pull.  
4. **Release** if wrench suddenly drops below `f_soft`.

---

## System Architecture

```text
┌──────────────────┐      ┌─────────────────┐      ┌─────────────────┐
│                  │      │                 │      │                 │
│  Servo Module    │─────▶│  Contact Logic  │─────▶│    MPC Planner  │
│  (PD to Handle)  │      │ (Force–Torque   │      │ (Sample & Cost) │
│                  │      │   thresholds)   │      │                 │
└──────────────────┘      └─────────────────┘      └─────────────────┘
           ▲                        │                       │
           │                        ▼                       │
           │                 Gripper Controller            │
           └────────────────────────────────────────────────┘
````

* **Input**: camera mask → handle frame, force/torque readings
* **Output**: 7 joint velocities + gripper command

---

## Installation & Deployment

### Prerequisites

* Ubuntu 18.04+ (or macOS/Linux with X11)
* Conda (Miniconda or Anaconda)
* MuJoCo 2.1 license & binaries installed per robosuite instructions

### Conda Environment Setup

```bash
conda create -n rsuite python=3.8
conda activate rsuite
```

### Clone & Install

```bash
git clone https://github.com/yourusername/7DOF-Cobot-FT-MPC-ArticulatingArm.git
cd 7DOF-Cobot-FT-MPC-ArticulatingArm

# Install Python dependencies
pip install -r requirements.txt
pip install -e .
```

---

## How to Customize

* **ForceTorqueLimits**: adjust soft/hard force & torque thresholds.
* **CostConfig**: tune weights for position, orientation, hinge angle, wrench.
* **ControlConfig**: modify PD gains, approach distance, grip levels, MPC horizon & samples.
* **Segmentation**: replace default camera to obtain handle mask from custom perception pipeline.

All parameters are exposed as CLI flags and in the `examples/run_door_controller.py` script.

---

## Cobots in Collaborative Robotics

Collaborative robots (cobots) like Sawyer, UR5e, and Franka Panda are designed to work safely alongside humans:

* **Lightweight arms** with torque sensing in every joint.
* **Force-torque sensors** at the wrist enable gentle, compliant behaviors.
* **Built-in safety**—easy integration of collision-detection logic.

This project demonstrates how to leverage these features for everyday tasks like door opening—bridging simulation proof-of-concept to real-world deployment.

---

## Future Scope

* **Real-world transfer**: integrate with ROS and Mujoco-ROS bridge for hardware experiments.
* **Vision-based grasping**: replace ideal segmentation mask with deep-learning handle detectors.
* **Adaptive MPC**: online learning of cost weights for new handle geometries.
* **Multi-object articulation**: extend to drawers, valves, and sliding doors.
* **Human–robot handoff**: detect human hand presence and dynamically yield control.

