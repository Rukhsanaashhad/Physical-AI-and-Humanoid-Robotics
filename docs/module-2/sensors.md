---
title: Sensor Simulation
description: LiDAR, Depth Camera, IMU simulation in Gazebo/Unity with example code.
slug: /module-2/sensors
---

# Sensor Simulation

Accurate sensor simulation is paramount for developing and testing robust robotic perception and control algorithms. It allows engineers to generate vast amounts of synthetic data for training machine learning models, evaluate sensor fusion techniques, and validate navigation and manipulation strategies without the need for expensive or fragile physical hardware. In the context of humanoid robotics, simulating sensors like LiDAR, depth cameras, and IMUs is critical for enabling the robot to "see," "feel," and "understand" its environment.

## LiDAR (Light Detection and Ranging) Simulation

LiDAR sensors provide a dense 3D point cloud of the environment, crucial for mapping, localization, and obstacle avoidance.

### Gazebo Simulation

Gazebo offers robust LiDAR simulation capabilities through its `sensor` tag in URDF or SDF. You can configure parameters like:

*   **Ray count**: Number of laser beams.
*   **Range**: Minimum and maximum detection distances.
*   **Horizontal/Vertical angles**: Field of view.
*   **Noise**: Adding realistic sensor noise.

**Example: LiDAR Sensor in URDF (Gazebo Plugin)**

```xml
<gazebo reference="base_link"> <!-- Attach to a link on your robot -->
  <sensor type="ray" name="lidar_sensor">
    <pose>0 0 0.1 0 0 0</pose> <!-- Relative to base_link -->
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.5708</min_angle> <!-- -90 degrees -->
          <max_angle>1.5708</max_angle>    <!-- +90 degrees -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>
    <plugin name="gazebo_ros_lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <argument>~/out</argument> <!-- ROS topic for point cloud data -->
        <argument>~/scan</argument> <!-- ROS topic for LaserScan data -->
      </ros>
      <output_type>sensor_msgs/PointCloud2</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

This configuration would publish `sensor_msgs/msg/PointCloud2` or `sensor_msgs/msg/LaserScan` messages to the specified ROS topics, just like a real LiDAR.

## Depth Camera Simulation

Depth cameras (e.g., Intel RealSense, Microsoft Azure Kinect) provide both RGB color images and per-pixel depth information, essential for 3D reconstruction, object detection, and navigation in complex environments.

### Gazebo/Unity Simulation

Both Gazebo and Unity can simulate depth cameras.

*   **Gazebo**: Uses the `camera` sensor type with the `depth` output option in URDF/SDF and typically employs `libgazebo_ros_depth_camera.so` plugin to publish `sensor_msgs/msg/Image` and `sensor_msgs/msg/PointCloud2` messages.
*   **Unity**: Can render depth textures directly from the camera, allowing for synthetic depth map generation. This is particularly useful for generating ground truth data for training.

**Example: Depth Camera in URDF (Gazebo Plugin)**

```xml
<gazebo reference="camera_link">
  <sensor type="depth" name="depth_camera">
    <always_on>true</always_on>
    <update_rate>30.0</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_depth_camera.so">
      <ros>
        <argument>~/image_raw</argument>       <!-- RGB topic -->
        <argument>~/depth/image_raw</argument> <!-- Depth topic -->
        <argument>~/points</argument>          <!-- Point cloud topic -->
      </ros>
      <camera_name>depth_camera_sensor</camera_name>
      <frame_name>camera_link_optical</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## IMU (Inertial Measurement Unit) Simulation

IMUs provide data on orientation, angular velocity, and linear acceleration, fundamental for robot state estimation, balance control, and navigation.

### Gazebo Simulation

Gazebo simulates IMUs through the `imu` sensor type and the `libgazebo_ros_imu_sensor.so` plugin. It takes into account the robot's motion and any external forces.

**Example: IMU Sensor in URDF (Gazebo Plugin)**

```xml
<gazebo reference="imu_link"> <!-- Attach to robot's torso or base_link -->
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100.0</update_rate>
    <imu>
      <orientation>
        <x>0</x>
        <y>0</y>
        <z>0</z>
      </orientation>
      <angular_velocity>
        <x>0</x>
        <y>0</y>
        <z>0</z>
      </angular_velocity>
      <linear_acceleration>
        <x>0</x>
        <y>0</y>
        <z>0</z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <argument>~/out</argument> <!-- ROS topic for IMU data -->
      </ros>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

Simulating these sensors effectively provides a powerful environment for developing and testing humanoid robotics algorithms, enabling rapid iteration and comprehensive validation.
