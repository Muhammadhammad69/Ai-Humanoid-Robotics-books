#!/usr/bin/env python3
"""
Whisper-to-ROS Integration Example

This script demonstrates how to integrate OpenAI Whisper with ROS 2 for voice command processing.
It creates a ROS 2 node that listens to audio input, processes it with Whisper,
and publishes recognized commands to ROS 2 topics for robot action execution.

Author: Robotics Developer
Date: 2025-12-10
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import AudioData
import whisper
import numpy as np
import threading
import queue
import time


class WhisperROSIntegration(Node):
    """
    ROS 2 node that integrates OpenAI Whisper for voice command processing.
    """

    def __init__(self):
        super().__init__('whisper_ros_integration')

        # Initialize Whisper model (using 'tiny' model for faster processing)
        self.get_logger().info('Loading Whisper model...')
        self.model = whisper.load_model("tiny")
        self.get_logger().info('Whisper model loaded successfully')

        # Audio processing queue
        self.audio_queue = queue.Queue(maxsize=100)

        # ROS 2 Publishers
        self.transcript_publisher = self.create_publisher(
            String,
            'whisper/transcript',
            10
        )

        self.command_publisher = self.create_publisher(
            String,
            'robot/command',
            10
        )

        self.status_publisher = self.create_publisher(
            String,
            'whisper/status',
            10
        )

        # ROS 2 Subscribers
        self.audio_subscriber = self.create_subscription(
            AudioData,
            'audio/input',
            self.audio_callback,
            10
        )

        self.control_subscriber = self.create_subscription(
            Bool,
            'whisper/enable',
            self.control_callback,
            10
        )

        # Parameters
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('audio_buffer_duration', 2.0)  # seconds
        self.declare_parameter('sample_rate', 16000)

        # State variables
        self.is_enabled = True
        self.audio_buffer = np.array([])
        self.buffer_duration = self.get_parameter('audio_buffer_duration').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.min_samples = int(self.buffer_duration * self.sample_rate)

        # Start processing thread
        self.processing_thread = threading.Thread(target=self.process_audio_stream, daemon=True)
        self.processing_thread.start()

        self.get_logger().info('Whisper-ROS Integration node initialized')

    def audio_callback(self, msg):
        """
        Callback function for audio input.
        """
        if not self.is_enabled:
            return

        try:
            # Convert audio data to numpy array (assuming 16-bit integer)
            audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

            # Add to processing queue
            self.audio_queue.put(audio_data, block=False)
        except queue.Full:
            self.get_logger().warn('Audio queue is full, dropping audio data')
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def control_callback(self, msg):
        """
        Callback function to enable/disable Whisper processing.
        """
        self.is_enabled = msg.data
        status_msg = String()
        status_msg.data = f'Whisper processing {"enabled" if self.is_enabled else "disabled"}'
        self.status_publisher.publish(status_msg)
        self.get_logger().info(status_msg.data)

    def process_audio_stream(self):
        """
        Continuously process audio from the queue.
        """
        while rclpy.ok():
            try:
                # Get audio data from queue
                audio_chunk = self.audio_queue.get(timeout=0.1)

                # Add to buffer
                self.audio_buffer = np.concatenate([self.audio_buffer, audio_chunk])

                # Process when buffer has enough data
                if len(self.audio_buffer) >= self.min_samples:
                    self.process_buffer()
                    # Keep some overlap for continuity
                    overlap_samples = int(0.5 * self.sample_rate)  # 0.5 second overlap
                    if len(self.audio_buffer) > overlap_samples:
                        self.audio_buffer = self.audio_buffer[-overlap_samples:]
                    else:
                        self.audio_buffer = np.array([])

            except queue.Empty:
                continue  # No audio data, continue waiting
            except Exception as e:
                self.get_logger().error(f'Error in audio stream processing: {e}')

    def process_buffer(self):
        """
        Process the accumulated audio buffer with Whisper.
        """
        if len(self.audio_buffer) == 0:
            return

        try:
            # Ensure audio is in the right format
            if len(self.audio_buffer) > 0:
                # Transcribe audio using Whisper
                result = self.model.transcribe(self.audio_buffer, language='en')

                transcript = result["text"].strip()
                confidence = result.get("avg_logprob", -1.0)

                # Publish transcript
                transcript_msg = String()
                transcript_msg.data = transcript
                self.transcript_publisher.publish(transcript_msg)

                # Check if confidence is above threshold
                min_conf = self.get_parameter('min_confidence').value
                if confidence > min_conf and transcript:
                    self.get_logger().info(f'Command recognized: "{transcript}" (confidence: {confidence:.2f})')

                    # Extract and publish robot command
                    robot_command = self.extract_robot_command(transcript)
                    if robot_command:
                        command_msg = String()
                        command_msg.data = robot_command
                        self.command_publisher.publish(command_msg)
                        self.get_logger().info(f'Published robot command: {robot_command}')
                else:
                    self.get_logger().info(f'Low confidence transcription: "{transcript}" (confidence: {confidence:.2f})')

        except Exception as e:
            self.get_logger().error(f'Error in Whisper transcription: {e}')

    def extract_robot_command(self, transcript):
        """
        Simple function to extract robot commands from transcript.
        In practice, this would use more sophisticated NLP.
        """
        transcript_lower = transcript.lower()

        # Define command mappings
        command_mappings = {
            'move forward': 'navigation_forward',
            'go forward': 'navigation_forward',
            'move backward': 'navigation_backward',
            'go backward': 'navigation_backward',
            'turn left': 'navigation_turn_left',
            'turn right': 'navigation_turn_right',
            'stop': 'navigation_stop',
            'pick up': 'manipulation_pick',
            'grasp': 'manipulation_pick',
            'place': 'manipulation_place',
            'put down': 'manipulation_place',
            'find': 'perception_find_object',
            'look for': 'perception_find_object',
            'bring me': 'manipulation_fetch',
            'get': 'manipulation_fetch'
        }

        # Check for command patterns
        for pattern, command in command_mappings.items():
            if pattern in transcript_lower:
                return command

        # If no specific command found, return a generic command
        if transcript.strip():
            return f'generic_command: {transcript}'

        return None

    def destroy_node(self):
        """
        Clean up resources when node is destroyed.
        """
        self.get_logger().info('Shutting down Whisper-ROS Integration node...')
        super().destroy_node()


def main(args=None):
    """
    Main function to run the Whisper-ROS Integration node.
    """
    rclpy.init(args=args)

    node = WhisperROSIntegration()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()