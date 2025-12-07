---
title: LLM Cognitive Planning
description: Translating natural language tasks into ROS 2 action sequences, example workflows.
slug: /module-4/llm-planning
---

# LLM Cognitive Planning

Large Language Models (LLMs) are revolutionizing how robots can understand and execute complex, high-level natural language commands. **LLM Cognitive Planning** refers to the use of LLMs to bridge the gap between abstract human instructions and the concrete, low-level action sequences required for a robot to perform a task, especially relevant for humanoid robots interacting in unstructured environments.

## The Challenge: Bridging the Semantic Gap

Human instructions are often ambiguous, underspecified, or assume common-sense knowledge. Robots, on the other hand, require precise, executable commands. The "semantic gap" is the challenge of translating human intent into robotic actions.

**Example**: A human might say, "Please clean up the living room."
For a robot, this involves:
1.  **Understanding "clean up"**: What constitutes "clean" (e.g., put toys in a bin, wipe surfaces)?
2.  **Identifying "living room"**: Locating the area and its boundaries.
3.  **Perceiving objects**: Finding toys, knowing their current state and desired state.
4.  **Planning a sequence**: How to move, grasp, place objects, avoid obstacles.

## LLMs for Action Sequence Generation

LLMs, with their vast knowledge base and ability to reason over text, can act as a high-level planner, translating human goals into a series of robot-executable sub-goals or actions.

### Workflow: Natural Language to ROS 2 Actions

1.  **User Input**: Natural language command (e.g., "Put the book on the shelf").
2.  **STT/Whisper**: Speech-to-Text converts audio to text (if voice input).
3.  **LLM Prompt Engineering**: The transcribed text is sent to an LLM with a carefully constructed prompt. This prompt defines the robot's capabilities (available ROS 2 actions/services), its current state, and the environment.
4.  **LLM Response**: The LLM generates a sequence of high-level actions (e.g., "Go to book", "Pick up book", "Go to shelf", "Place book").
5.  **Action Grounding**: Each high-level action is then "grounded" into a specific ROS 2 action call, service request, or a series of low-level commands. This often involves:
    *   **Perception**: Identifying objects and their poses.
    *   **Motion Planning**: Generating collision-free trajectories.
    *   **Execution**: Sending commands to robot controllers.
6.  **Robot Execution**: The ROS 2 action client executes the sequence.

### Example: LLM Prompt for a Humanoid Robot (Conceptual)

```
---
Role: You are a helpful assistant that translates natural language commands for a humanoid robot.
The robot has the following capabilities (ROS 2 actions/services):
- GOTO_POSE(x, y, z, roll, pitch, yaw): Move to a 3D pose.
- PICK_OBJECT(object_name): Grasp a specified object.
- PLACE_OBJECT(location_name): Place the held object at a location.
- FIND_OBJECT(object_name): Return the pose of an object.
- REPORT_STATUS(): Report current battery and operational status.

Current Robot State:
- Holding: Nothing
- Location: Living room
- Nearby Objects: Red book (at 0.5, 0.2, 0.8), Blue cup (at 1.0, -0.3, 0.7)
---

Human Command: "Can you grab the red book and put it on the table?"

Robot Thought Process:
1. Identify intent: Grab and place.
2. Identify objects: "red book".
3. Identify target location: "table" (assume table is a known location or can be found by FIND_OBJECT).
4. Generate action sequence:
    - FIND_OBJECT(red book)
    - GOTO_POSE(pose of red book)
    - PICK_OBJECT(red book)
    - GOTO_POSE(pose of table)
    - PLACE_OBJECT(table)

LLM Response (structured, for parsing by robot control system):
```json
{
  "plan": [
    {"action": "FIND_OBJECT", "args": ["red book"]},
    {"action": "GOTO_POSE", "args": ["pose_of_red_book"]},
    {"action": "PICK_OBJECT", "args": ["red book"]},
    {"action": "GOTO_POSE", "args": ["pose_of_table"]},
    {"action": "PLACE_OBJECT", "args": ["table"]}
  ],
  "confidence": 0.95
}
```

## Considerations for Humanoid Robots

*   **Embodiment**: LLMs need to understand the physical constraints and capabilities of the humanoid (e.g., can it reach, grasp, balance?).
*   **Common Sense**: Injecting real-world common sense to avoid nonsensical plans (e.g., "pick up the table").
*   **Error Recovery**: LLMs can also assist in generating recovery plans when an action fails.
*   **Safety**: Ensuring that LLM-generated plans adhere to safety protocols and do not put the robot or humans at risk.
*   **Tool Use**: LLMs can be taught to use specific ROS 2 tools (actions, services) as their "functions" to achieve goals.

LLM Cognitive Planning represents a significant leap towards more intuitive and flexible human-robot collaboration, allowing humanoids to perform complex tasks with high-level guidance.
