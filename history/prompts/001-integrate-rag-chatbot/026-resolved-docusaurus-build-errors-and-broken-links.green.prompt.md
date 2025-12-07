---
id: 026
title: Resolved Docusaurus build errors and broken links
stage: green
date_iso: 2025-12-07
model: gemini-1.5-pro
feature: 001-integrate-rag-chatbot
branch: 001-integrate-rag-chatbot
user: null
command: |-
  I have a Docusaurus v3 project that fails to build due to broken links and missing sidebar doc IDs.
  Here are the errors:

  1. Broken links:
  - /Physical-AI-and-Humanoid-Robotics/docs/05-hardware links to:
    - workstation-specs
    - jetson-deployment
  - /Physical-AI-and-Humanoid-Robotics/docs/06-capstone links to:
    - sim-to-real-deployment

  2. Sidebar errors (these doc IDs do not exist):
  - capstone/architecture
  - capstone/capstone-lab
  - capstone/deployment-notes
  - capstone/evaluation
  - capstone/overview
  - hardware/jetson
  - hardware/lab-setup
  - hardware/overview
  - hardware/robot-platforms
  - hardware/workstation

  Available document IDs (existing docs) are:
  - index
  - intro/index
  - intro/intro
  - intro/spacekit-example
  - module-1/ros2-nodes
  - module-1/ros2-overview
  - module-1/ros2-topics-services
  - module-1/urdf-humanoids
  - module-2/gazebo-physics
  - module-2/sensors
  - module-2/unity-rendering
  - module-3/isaac-ros
  - module-3/isaac-sim
  - module-3/nav2
  - module-4/llm-planning
  - module-4/vla-architecture
  - module-4/whisper
  - sim-to-real-deployment/architecture
  - sim-to-real-deployment/capstone-lab
  - sim-to-real-deployment/deployment-notes
  - sim-to-real-deployment/evaluation
  - sim-to-real-deployment/overview
  - workstation-specs/jetson
  - workstation-specs/lab-setup
  - workstation-specs/overview
  - workstation-specs/robot-platforms
  - workstation-specs/workstation

  Please:
  1. Update the `sidebars.ts` file to fix all missing doc IDs using the available documents.
  2. Update any broken links in the markdown files to match the available document IDs.
  3. Ensure the Docusaurus project can build successfully for locale "en" and be ready for deployment.
  4. Keep the existing folder structure and only map sidebar items to valid docs.
labels:
  - docusaurus-fix
  - broken-links
  - sidebar-fix
  - config-update
  - caching
links: null
files_yaml:
  - docusaurus.config.ts
  - sidebars.ts
  - docs/workstation-specs/04-lab-setup.md
  - docs/workstation-specs/03-robot-platforms.md
  - docs/workstation-specs/02-jetson.md
  - docs/workstation-specs/01-workstation.md
  - docs/workstation-specs/00-overview.md
  - docs/sim-to-real-deployment/04-capstone-lab.md
  - docs/sim-to-real-deployment/03-deployment-notes.md
  - docs/sim-to-real-deployment/02-evaluation.md
  - docs/sim-to-real-deployment/01-architecture.md
  - docs/sim-to-real-deployment/00-overview.md
tests_yaml: null
prompt_text: |-
  I have a Docusaurus v3 project that fails to build due to broken links and missing sidebar doc IDs.
  Here are the errors:

  1. Broken links:
  - /Physical-AI-and-Humanoid-Robotics/docs/05-hardware links to:
    - workstation-specs
    - jetson-deployment
  - /Physical-AI-and-Humanoid-Robotics/docs/06-capstone links to:
    - sim-to-real-deployment

  2. Sidebar errors (these doc IDs do not exist):
  - capstone/architecture
  - capstone/capstone-lab
  - capstone/deployment-notes
  - capstone/evaluation
  - capstone/overview
  - hardware/jetson
  - hardware/lab-setup
  - hardware/overview
  - hardware/robot-platforms
  - hardware/workstation

  Available document IDs (existing docs) are:
  - index
  - intro/index
  - intro/intro
  - intro/spacekit-example
  - module-1/ros2-nodes
  - module-1/ros2-overview
  - module-1/ros2-topics-services
  - module-1/urdf-humanoids
  - module-2/gazebo-physics
  - module-2/sensors
  - module-2/unity-rendering
  - module-3/isaac-ros
  - module-3/isaac-sim
  - module-3/nav2
  - module-4/llm-planning
  - module-4/vla-architecture
  - module-4/whisper
  - sim-to-real-deployment/architecture
  - sim-to-real-deployment/capstone-lab
  - sim-to-real-deployment/deployment-notes
  - sim-to-real-deployment/evaluation
  - sim-to-real-deployment/overview
  - workstation-specs/jetson
  - workstation-specs/lab-setup
  - workstation-specs/overview
  - workstation-specs/robot-platforms
  - workstation-specs/workstation

  Please:
  1. Update the `sidebars.ts` file to fix all missing doc IDs using the available documents.
  2. Update any broken links in the markdown files to match the available document IDs.
  3. Ensure the Docusaurus project can build successfully for locale "en" and be ready for deployment.
  4. Keep the existing folder structure and only map sidebar items to valid docs.
response_text: |-
  All Docusaurus v3 build issues, including broken links, invalid sidebar IDs, case sensitivity in markdown links, and the `onBrokenMarkdownLinks` deprecation warning, have been successfully resolved. The Docusaurus project now builds successfully.

  Specifically:
  -   `docusaurus.config.ts` was updated to correctly place the `markdown.hooks.onBrokenMarkdownLinks` configuration and other `markdown` properties.
  -   `sidebars.ts` was confirmed to correctly reference existing doc IDs under `workstation-specs/` and `sim-to-real-deployment/`.
  -   Markdown files (`docs/workstation-specs/00-overview.md` and `docs/sim-to-real-deployment/00-overview.md`) were confirmed to have correct `slug` front matter and internal links. The previous build errors were resolved by clearing the Docusaurus cache (`npm run clear`).

  The Docusaurus project now builds successfully, confirming full resolution of the reported issues.
outcome: All Docusaurus v3 build issues, including broken links, invalid sidebar IDs, markdown link case sensitivity, and `onBrokenMarkdownLinks` deprecation, have been successfully resolved, leading to a successful build.
evaluation: The request was fully addressed by systematically identifying and fixing each reported issue across docusaurus.config.ts, sidebars.ts, and relevant markdown files. The successful build confirms the effectiveness of the applied changes, and the cache clearing addressed the transient issues.
---