---
title: URDF for Humanoids
description: Unified Robot Description Format for humanoid robot structure, joints, and sensors.
slug: /module-1/urdf-humanoids
---

# URDF for Humanoids

The **Unified Robot Description Format (URDF)** is an XML format used in ROS to describe all aspects of a robot. It's crucial for robotic applications, enabling visualization, simulation, and planning algorithms to understand the robot's physical properties, kinematics, and dynamics. For humanoid robots, URDF becomes even more critical due to their complex articulated structures and need for precise modeling.

## Core URDF Elements

A URDF file primarily defines two types of elements:

1.  **`<link>`**: Represents a rigid body part of the robot (e.g., torso, upper arm, forearm, hand). Links have physical properties such as mass, inertia, visual appearance, and collision geometry.
2.  **`<joint>`**: Connects two links, defining their relative motion. Joints have properties like type (revolute, prismatic, fixed), axis of rotation, limits of motion, and dynamic characteristics.

### Example: Basic Humanoid Arm Segment (Simplified)

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid_arm">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <link name="shoulder_link">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="0.5"/>
  </joint>

  <link name="upper_arm_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.03"/>
      </geometry>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="elbow_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0" effort="10" velocity="0.5"/>
  </joint>

</robot>
```

## Special Considerations for Humanoids

Humanoid robots are characterized by:

*   **High Degrees of Freedom (DoF)**: Many joints in legs, arms, and torso require detailed modeling.
*   **Balancing and Locomotion**: Accurate mass and inertia properties are crucial for stable bipedal walking and complex movements.
*   **Self-Collision**: Due to their human-like form, self-collision detection (e.g., an arm hitting the torso) must be carefully configured using collision geometry.
*   **Sensor Integration**: URDF can describe the placement and type of sensors (e.g., cameras, LiDAR, IMU) attached to different links, which is vital for perception and state estimation.
*   **Actuators**: While URDF itself doesn't define actuators, it provides the joint limits and dynamics properties that external control systems (like `ros2_control`) use to interact with physical motors.

### URDF Extensions (Xacro)

Writing complex URDF files manually can be tedious and prone to errors, especially for humanoids with repetitive structures (e.g., fingers, segments of legs). **Xacro (XML Macros)** is an XML macro language that allows for more concise and readable URDF descriptions by enabling:

*   **Macros**: Reusing common patterns for links and joints.
*   **Parameters**: Defining configurable values (e.g., segment lengths, joint limits).
*   **Mathematical Expressions**: Performing calculations within the URDF.

Xacro greatly simplifies the creation and maintenance of URDF for highly articulated robots.

## Visual and Collision Geometry

*   **`<visual>`**: Defines how the link appears in visualization tools (like RViz). This can reference mesh files (e.g., `.stl`, `.dae`) for realistic rendering.
*   **`<collision>`**: Defines the geometry used for collision detection in simulators (like Gazebo). This is often simplified geometry (e.g., boxes, spheres, cylinders) to reduce computational load.

For humanoids, ensuring that visual and collision geometries are accurately defined for each body part is essential for both realistic simulation and preventing unwanted self-collisions or collisions with the environment.

Mastering URDF is a foundational skill for anyone working with humanoid robots, as it forms the blueprint for how the robot is understood and interacted with by the ROS ecosystem.
