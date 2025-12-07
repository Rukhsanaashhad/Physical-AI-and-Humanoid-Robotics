---
title: Whisper Voice Commands
description: Voice-to-Action pipeline for humanoid robots, integration with ROS 2.
slug: /module-4/whisper
---

# Module 4: Whisper Voice Commands

Integrating natural language understanding into humanoid robots opens up new avenues for human-robot interaction, making robots more intuitive and accessible. **OpenAI's Whisper** is a powerful general-purpose speech-to-text model that can accurately transcribe human speech into text, forming a crucial first step in a voice-to-action pipeline for humanoid robots.

## The Voice-to-Action Pipeline

A typical voice-to-action pipeline for a humanoid robot involves several stages:

1.  **Speech Capture**: The robot's microphones capture ambient speech.
2.  **Speech-to-Text (STT)**: A model like Whisper transcribes the audio into text.
3.  **Natural Language Understanding (NLU)**: The transcribed text is processed to extract intent and relevant entities (e.g., "move forward," "pick up the red cube," "go to the kitchen").
4.  **Action Planning**: Based on the NLU output, the robot's control system plans a sequence of robotic actions.
5.  **Robot Execution**: The planned actions are executed by the robot's actuators.
6.  **Feedback**: The robot provides auditory or visual feedback to the human.

Whisper primarily handles step 2, providing highly accurate and robust transcription even in noisy environments or with different accents.

## Integrating Whisper with ROS 2

Integrating Whisper into a ROS 2 ecosystem typically involves creating a ROS 2 node that:

*   Subscribes to an audio topic (e.g., from the robot's microphone array).
*   Processes the audio data using the Whisper model (either locally or via an API).
*   Publishes the transcribed text to another ROS 2 topic.

### Example: Whisper ROS 2 Node (Conceptual Python)

This conceptual example shows a ROS 2 node that would receive audio and publish transcribed text.

```python
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData # Assuming an audio message type
from std_msgs.msg import String
# import whisper # Assuming whisper library is installed

class WhisperSTTNode(Node):
    def __init__(self):
        super().__init__('whisper_stt_node')
        self.subscription = self.create_subscription(
            AudioData,
            '/audio_in',
            self.audio_callback,
            10
        )
        self.publisher_ = self.create_publisher(String, '/transcribed_text', 10)
        self.get_logger().info('Whisper STT Node started, awaiting audio...')

        # self.whisper_model = whisper.load_model("base") # Load a Whisper model

    def audio_callback(self, msg: AudioData):
        self.get_logger().info('Received audio data.')
        # Assuming msg.data contains raw audio bytes
        # For actual Whisper use, audio needs to be in a specific format (e.g., WAV, 16kHz mono)

        # Conceptual: Save audio to a temporary file or process in-memory
        # audio_segment = self.convert_raw_to_whisper_format(msg.data) 
        # result = self.whisper_model.transcribe(audio_segment)
        # transcribed_text = result["text"]

        transcribed_text = "Simulated transcription: Move forward" # Placeholder for actual transcription

        if transcribed_text:
            text_msg = String()
            text_msg.data = transcribed_text
            self.publisher_.publish(text_msg)
            self.get_logger().info(f'Transcribed: "{transcribed_text}"')

    # def convert_raw_to_whisper_format(self, raw_audio_data):
    #     # Placeholder for actual audio processing logic (e.g., resampling, format conversion)
    #     return raw_audio_data

def main(args=None):
    rclpy.init(args=args)
    node = WhisperSTTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Considerations for Humanoid Robots

*   **Microphone Array**: Humanoids typically have multiple microphones for better sound localization and noise cancellation. Pre-processing audio from these arrays can improve Whisper's accuracy.
*   **Noise Robustness**: Robots often operate in noisy environments. Whisper's robustness to noise is a significant advantage.
*   **Latency**: For real-time interaction, the latency of the STT process needs to be minimized.
*   **Contextual Understanding**: While Whisper provides text, the subsequent NLU stage is critical for the robot to understand the *meaning* and *intent* behind the voice commands in the context of its capabilities and environment.

By effectively implementing Whisper, humanoid robots can move beyond basic teleoperation to more natural, voice-driven interfaces, enhancing their utility and user experience in various applications.
