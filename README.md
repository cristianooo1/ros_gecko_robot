## ROS 2 Jazzy Quadruped Robot – Knowledge Block

## Robot Overview

- Quadrupedal robot with 4 legs (FR, FL, BR, BL).

- Each leg has 2 joints: knee and ankle.

Joint limits are asymmetric and defined in the URDF/Xacro:

- Knees: some [-1.5708, 0], some [0, 1.5708]

- Ankles: [0, 0.872665]

## Joint Interfaces

- Command interfaces: position only

- State interfaces: position, velocity, effort

- Provides feedback for controllers and monitoring

## ROS 2 Controllers

- Controller Manager: handles claiming and activating joints

Controllers used:

- JointGroupPositionController – allows direct position commands to multiple joints

- JointTrajectoryController – accepts FollowJointTrajectory action goals for sending trajectories

Joints are successfully claimed and controlled by the controllers

Teleoperation Node (Python):

Keyboard-based control of individual legs:

- n – cycle active leg

- u/d – move ankle up/down

- l/r – move knee left/right

Uses FollowJointTrajectory action client to send incremental single-point trajectories to the JointTrajectoryController

Subscribes to /joint_states for feedback: prints joint positions periodically (currently 1 Hz)

Ensures all commands are clamped to joint limits

## Current Achievements

- Configured hardware interfaces and joint limits

- Successfully activated controllers and claimed joints

- Implemented teleoperation node that controls any leg/joint with keyboard commands

- Real-time monitoring of joint positions via state interfaces

- Supports incremental joint movements with single-point trajectories

## Potential Future Extensions

- GAIT PLANNER!!!!!!!!!!!!!

- Continuous motion when holding keys

- Multi-joint coordinated trajectories
