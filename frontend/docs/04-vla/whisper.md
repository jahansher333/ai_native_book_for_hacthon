---
id: whisper
title: Speech-to-Text with Whisper
sidebar_label: Whisper
sidebar_position: 2
description: Deploy OpenAI Whisper on Jetson Orin Nano for real-time speech recognition
keywords: [whisper, speech-to-text, stt, voice-recognition, jetson]
---

# Speech-to-Text with Whisper

## What is Whisper?

**Whisper** is OpenAI's state-of-the-art speech-to-text model, supporting 99 languages with high accuracy even in noisy environments.

### Model Sizes

| Model | Parameters | VRAM | Speed (Jetson Orin Nano) |
|-------|-----------|------|--------------------------|
| **Tiny** | 39M | 1GB | 10x real-time (fast) |
| **Base** | 74M | 1GB | 7x real-time |
| **Small** | 244M | 2GB | 4x real-time (recommended) |
| **Medium** | 769M | 5GB | 2x real-time |
| **Large** | 1550M | 10GB | 1x real-time (too slow for Jetson) |

**Recommendation**: Use **Small** model for balance of accuracy and speed on Jetson.

---

## Installation

### Install Whisper

```bash
# On Jetson Orin Nano
pip3 install openai-whisper
```

### Install Audio Dependencies

```bash
sudo apt install -y portaudio19-dev python3-pyaudio
pip3 install pyaudio
```

---

## Basic Whisper Usage

### Transcribe Audio File

```python
import whisper

# Load model
model = whisper.load_model("small")

# Transcribe audio file
result = model.transcribe("audio.wav")

print(result["text"])
# Output: "Hello, how can I help you?"
```

---

## Real-Time Microphone Transcription

```python
import whisper
import pyaudio
import wave
import numpy as np

class WhisperSTT:
    def __init__(self, model_size="small"):
        self.model = whisper.load_model(model_size)

        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=1024
        )

    def record_audio(self, duration=5):
        """Record audio for specified duration (seconds)"""
        print(f"Recording for {duration} seconds...")

        frames = []
        for _ in range(0, int(16000 / 1024 * duration)):
            data = self.stream.read(1024)
            frames.append(data)

        print("Recording complete")

        # Convert to NumPy array
        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
        audio_data = audio_data.astype(np.float32) / 32768.0  # Normalize

        return audio_data

    def transcribe(self, audio_data):
        """Transcribe audio data"""
        result = self.model.transcribe(audio_data, language='en')
        return result["text"]

    def listen(self, duration=5):
        """Record and transcribe"""
        audio_data = self.record_audio(duration)
        text = self.transcribe(audio_data)
        return text

# Usage
stt = WhisperSTT(model_size="small")
text = stt.listen(duration=5)
print(f"You said: {text}")
```

---

## ROS 2 Integration

### Voice Command Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import pyaudio
import numpy as np

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Publisher for transcribed text
        self.command_pub = self.create_publisher(String, '/voice_command', 10)

        # Load Whisper model
        self.model = whisper.load_model("small")

        # Setup audio stream
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=1024
        )

        # Timer to check for voice activity
        self.create_timer(1.0, self.check_voice_activity)

        self.get_logger().info('Voice command node ready')

    def record_audio(self, duration=3):
        """Record audio for duration seconds"""
        frames = []
        for _ in range(0, int(16000 / 1024 * duration)):
            data = self.stream.read(1024, exception_on_overflow=False)
            frames.append(data)

        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
        audio_data = audio_data.astype(np.float32) / 32768.0

        return audio_data

    def check_voice_activity(self):
        """Detect voice activity and transcribe"""
        # Simple energy-based voice activity detection
        frames = []
        for _ in range(5):  # Sample 5 frames
            data = self.stream.read(1024, exception_on_overflow=False)
            frames.append(data)

        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
        energy = np.abs(audio_data).mean()

        if energy > 500:  # Threshold (tune for your environment)
            self.get_logger().info('Voice detected! Recording...')

            # Record full command
            full_audio = self.record_audio(duration=3)

            # Transcribe
            result = self.model.transcribe(full_audio, language='en')
            text = result["text"].strip()

            if text:
                self.get_logger().info(f'Transcribed: {text}')

                # Publish command
                msg = String()
                msg.data = text
                self.command_pub.publish(msg)

def main():
    rclpy.init()
    node = VoiceCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## ReSpeaker Mic Array Integration

For far-field voice detection (4-mic array):

```bash
# Install ReSpeaker driver
pip3 install pyusb click
git clone https://github.com/respeaker/usb_4_mic_array.git
cd usb_4_mic_array
python3 doa.py  # Test direction of arrival
```

### Use with Whisper

```python
import usb.core
import usb.util

class ReSpeakerWhisper:
    def __init__(self):
        self.model = whisper.load_model("small")

        # Find ReSpeaker device
        self.dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)

        if self.dev is None:
            raise ValueError("ReSpeaker not found")

        # Setup audio stream (similar to PyAudio)
        # ... (device-specific setup)

    def get_direction(self):
        """Get direction of arrival (DOA) in degrees"""
        # Query ReSpeaker for DOA
        direction = self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0,
            0x0400,
            0,
            1
        )[0]

        return direction

    def listen_directional(self):
        """Listen and get direction of speaker"""
        direction = self.get_direction()
        audio_data = self.record_audio()
        text = self.model.transcribe(audio_data)["text"]

        return text, direction
```

---

## Confidence Scores

Check transcription confidence:

```python
result = model.transcribe(audio_data, language='en', word_timestamps=True)

# Average log probability (confidence)
confidence = np.exp(result["segments"][0]["avg_logprob"])

if confidence < 0.8:
    print("Low confidence, ask user to repeat")
else:
    print(f"Transcribed: {result['text']}")
```

---

## Hands-On Lab: Voice-Controlled Robot

**Goal**: Control robot with voice commands.

### Requirements

1. ROS 2 node that listens for commands
2. Recognize commands: "forward", "backward", "turn left", "turn right", "stop"
3. Publish velocity commands to `/cmd_vel`
4. Use Whisper Small model on Jetson

### Starter Code

```python
class VoiceControlledRobot(Node):
    def __init__(self):
        super().__init__('voice_controlled_robot')

        self.voice_sub = self.create_subscription(
            String, '/voice_command', self.command_callback, 10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def command_callback(self, msg):
        command = msg.data.lower()

        twist = Twist()

        if 'forward' in command:
            twist.linear.x = 0.5
        elif 'backward' in command:
            twist.linear.x = -0.5
        elif 'left' in command:
            twist.angular.z = 0.5
        elif 'right' in command:
            twist.angular.z = -0.5
        elif 'stop' in command:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'Executed: {command}')
```

---

## Key Takeaways

✅ **Whisper** provides accurate speech-to-text on edge devices
✅ **Small model** best for Jetson Orin Nano (balance speed/accuracy)
✅ **ReSpeaker** for far-field voice detection
✅ **Check confidence scores** before executing commands

---

## Next Steps

Learn LLM-based planning in **[Chapter 2: LLM Planning](/docs/vla/llm-planning)**.

<div style={{textAlign: 'center', marginTop: '3rem', padding: '2rem', backgroundColor: 'var(--ifm-color-emphasis-100)', borderRadius: '8px'}}>
  <h2>📚 Continue Learning</h2>
  <div style={{marginTop: '2rem'}}>
    <a className="button button--primary button--lg" href="/docs/vla/llm-planning">
      Next: LLM Planning →
    </a>
  </div>
</div>
