---
title: Gazebo Physics Simulation
description: Simulating humanoid motion, gravity, collisions, environment setup, physics examples.
slug: /module-2/gazebo-physics
---

# Module 2: Gazebo Physics Simulation

**Gazebo** is a powerful 3D robotics simulator that allows you to accurately and efficiently simulate complex robot scenarios in challenging indoor and outdoor environments. It's widely used in the ROS community for testing algorithms, designing robots, and conducting research without requiring physical hardware. For humanoid robots, Gazebo's realistic physics engine is essential for simulating bipedal locomotion, balance, and human-robot interaction.

## Simulating Humanoid Motion

Simulating humanoid motion in Gazebo involves defining the robot's URDF (Unified Robot Description Format) with accurate inertial properties, joint limits, and controller configurations. Gazebo interprets this URDF to construct the robot model and apply physics.

Key aspects for humanoid motion:

*   **URDF Accuracy**: Precise mass, inertia, and collision geometries for each link are critical for stable and realistic bipedal movement.
*   **Joint Controllers**: Using ROS 2 `ros2_control` with `effort_controllers` or `position_controllers` to command the humanoid's joints.
*   **Balancing**: Implementing feedback control loops (e.g., PID controllers) to maintain balance during walking or complex poses.
*   **Foot Contact**: Simulating realistic ground contact interactions for stable locomotion.

## Gravity and Collisions

Gazebo's physics engine (e.g., ODE, Bullet, DART, Simbody) accurately simulates fundamental physical phenomena:

*   **Gravity**: Applied to all links of the robot and objects in the environment, affecting balance and trajectory.
*   **Collisions**: Detected between robot links and environmental objects. Accurate collision geometries in the URDF are vital.

### Physics Examples: Falling Humanoid (Conceptual)

Consider a simple scenario where a humanoid robot loses balance and falls. Gazebo would simulate:

1.  **Initial State**: Robot standing, gravity acting downwards.
2.  **Loss of Balance**: An external force or an unstable pose causes the robot's center of mass to move outside its support polygon.
3.  **Gravitational Pull**: Gravity causes the robot to accelerate downwards.
4.  **Impact**: Collision detection registers contact with the ground.
5.  **Resting State**: The robot settles on the ground, potentially damaged depending on collision parameters.

This allows engineers to test fall recovery mechanisms or design more robust gaits.

## Environment Setup

Creating realistic environments in Gazebo is crucial for comprehensive simulation.

*   **SDF (Simulation Description Format)**: Gazebo primarily uses SDF to describe worlds, including terrain, buildings, obstacles, light sources, and other static and dynamic objects.
*   **Assets**: Importing 3D models (meshes) for complex objects (e.g., furniture, tools, human models) to populate the environment.
*   **Sensors**: Adding virtual sensors (cameras, LiDAR, IMU) to the environment or directly to the robot model, configured to provide realistic data streams.

### Example: Simple Gazebo World with an Obstacle

This is a simplified SDF snippet to create a flat ground plane and a box obstacle.

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <model name="my_obstacle">
      <pose>1 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.1 0.1 1</ambient>
            <diffuse>0.8 0.1 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```
To run this, you would save it as `.world` file and then launch Gazebo with `gazebo your_world_file.world`.

## Physics Examples

*   **Inverse Kinematics (IK) Validation**: Simulating whether a humanoid's arm can reach a target point given joint limits and collision constraints.
*   **Dynamic Walking Gaits**: Developing and testing different walking patterns, analyzing stability and energy consumption.
*   **Manipulation Tasks**: Simulating how a humanoid can grasp and manipulate objects, considering friction, weight, and object geometry.

Gazebo provides an invaluable sandbox for iterating on complex humanoid behaviors and designs before deploying them to expensive and fragile hardware.
