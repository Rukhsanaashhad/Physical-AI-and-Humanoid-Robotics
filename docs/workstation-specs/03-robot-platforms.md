---
title: Robot Platforms
description: Overview of various robot hardware platforms for humanoid robotics.
slug: /workstation-specs/03-robot-platforms
---

# Robot Platforms

This section provides an overview of various robot hardware platforms commonly used in humanoid robotics research and development, beyond just Jetson devices. It covers different types of humanoids, their unique characteristics, and considerations for choosing a platform for specific research or application needs.

## Types of Humanoid Robot Platforms

1.  **Full-sized Humanoids**:
    *   **Description**: Robots designed to mimic human morphology and capabilities, often used in advanced research for complex manipulation, locomotion, and human-robot interaction.
    *   **Examples**: Boston Dynamics Atlas, Digit (Agility Robotics), HRP-5P.
    *   **Considerations**: High cost, complex control, significant power requirements, advanced safety protocols.

2.  **Small to Medium-sized Humanoids**:
    *   **Description**: More accessible platforms for academic research and smaller-scale applications, often focusing on specific aspects like balance, gait, or interaction.
    *   **Examples**: DARwIn-OP, Robotis OP3, Unitree H1.
    *   **Considerations**: Relatively lower cost, easier to experiment with, good for prototyping algorithms.

3.  **Modular Robot Systems**:
    *   **Description**: Platforms built from reconfigurable modules, allowing researchers to experiment with different robot morphologies, including humanoid-like structures.
    *   **Examples**: Dynamixel-based robots, custom 3D-printed designs.
    *   **Considerations**: Flexibility in design, requires significant integration effort.

## Key Selection Criteria

When choosing a humanoid robot platform, consider:

*   **Cost**: Budget constraints often dictate available options.
*   **Degrees of Freedom (DoF)**: The number of controllable joints, influencing the robot's dexterity and movement complexity.
*   **Actuation**: Type of motors (e.g., servo, hydraulic, electric), their power, torque, and precision.
*   **Sensors**: Onboard sensors (cameras, IMU, force sensors, LiDAR) and their capabilities.
*   **Software Ecosystem**: Compatibility with ROS 2, available drivers, SDKs, and community support.
*   **Durability and Maintenance**: Robustness for research environments and ease of repair.
*   **Safety Features**: Emergency stops, compliance, and other safety mechanisms.

## Future Trends

The field is continuously evolving with:

*   **Open-source hardware designs**: Lowering the barrier to entry.
*   **Advanced materials**: Lighter and stronger robot structures.
*   **More integrated compute**: Onboard processing power for advanced AI.

Choosing the right platform is a critical decision that impacts the scope and feasibility of your humanoid robotics projects.
