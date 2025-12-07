---
title: Jetson Deployment
description: Guide to deploying robotics applications on NVIDIA Jetson platforms.
slug: /05-hardware/jetson-deployment
---

# Deploying to NVIDIA Jetson Platforms

This guide provides an overview of deploying your robotics applications, particularly those developed with ROS 2 and AI models, to NVIDIA Jetson edge AI platforms (e.g., Jetson Nano, Xavier NX, AGX Orin).

## Key Steps:

1.  **Hardware Setup**: Initial setup of your Jetson device, including flashing the OS (JetPack SDK).
2.  **Environment Configuration**: Setting up the development environment, including ROS 2, CUDA, cuDNN, and other necessary libraries.
3.  **Cross-Compilation/Deployment**: Strategies for building and deploying your applications to the Jetson.
4.  **Optimization**: Tips for optimizing your applications for performance on the Jetson's constrained resources.

## Best Practices:
- Always use the latest stable JetPack SDK.
- Leverage NVIDIA's optimized libraries (e.g., TensorRT) for AI inference.
- Monitor resource usage (CPU, GPU, memory) during deployment.
