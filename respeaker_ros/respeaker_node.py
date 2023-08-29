#!/usr/bin/env python3
import usb.core
import usb.util

import rclpy
from rclpy.node import Node
import rclpy.time
import rclpy.duration
import rclpy.timer
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


from audio_common_msgs.msg import AudioData
from std_msgs.msg import UInt16, Int32, Bool, ColorRGBA
from geometry_msgs.msg import PoseStamped

from respeaker_ros.interface import RespeakerInterface
from respeaker_ros.audio import RespeakerAudio
import math
import numpy as np
import angles
import tf_transformations as T


class RespeakerNode(Node):
    def __init__(self):
        super().__init__('respeaker_node')

        self.sensor_frame_id = self.declare_parameter('sensor_frame_id', 'respeaker_base')
        self.speech_prefetch = self.declare_parameter('speech_prefetch', 0.5)
        self.update_period_s = self.declare_parameter('update_period_s', 0.1)
        self.main_channel = self.declare_parameter('main_channel', 0)
        self.speech_continuation = self.declare_parameter('speech_continuation', 0.5)
        self.speech_max_duration = self.declare_parameter('speech_max_duration', 7.0)
        self.speech_min_duration = self.declare_parameter('speech_min_duration', 0.1)
        self.doa_xy_offset = self.declare_parameter('doa_xy_offset', 0.0)
        self.doa_yaw_offset = self.declare_parameter('doa_yaw_offset', 90.0)

        self.respeaker = RespeakerInterface()
        self.respeaker_audio = RespeakerAudio(self.on_audio, suppress_error=True)

        self.speech_audio_buffer = []
        self.is_speaking = False
        self.speech_stopped = self.get_clock().now()
        self.prev_is_voice = None
        self.prev_doa = None
        latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_vad = self.create_publisher(Bool, 'vad', latching_qos)
        self._pub_doa_raw = self.create_publisher(Int32, 'doa_raw', latching_qos)
        self._pub_doa = self.create_publisher(PoseStamped, 'doa', latching_qos)
        self._pub_audio = self.create_publisher(AudioData, 'audio', 10)
        self._pub_audio_channels = {c: self.create_publisher(AudioData, 'audio/channel%d' % c, 10) for c in self.respeaker_audio.channels}
        self._pub_speech_audio = self.create_publisher(AudioData, 'speech_audio', 10)
        self._timer = self.create_timer(self.update_period_s.value, self.on_timer)

        self.speech_prefetch_bytes = int(
            self.speech_prefetch.value * self.respeaker_audio.rate * self.respeaker_audio.bitdepth / 8.0)
        self.speech_prefetch_buffer = np.zeros(self.speech_prefetch_bytes, dtype=np.uint8)
        self.respeaker_audio.start()
 
        self.timer_led = None
        self.sub_led = self.create_subscription(ColorRGBA, "status_led", self.on_status_led, 1)

    def on_audio(self, data, channel):
        as_uint8 = data.astype(np.uint8)
        channel_pub = self._pub_audio_channels[channel]
        if channel_pub.get_subscription_count() > 0:
            channel_pub.publish(AudioData(data=as_uint8))
        if channel == self.main_channel.value:
            if self._pub_audio.get_subscription_count() > 0:
                self._pub_audio.publish(AudioData(data=as_uint8))
            if self.is_speaking:
                if len(self.speech_audio_buffer) == 0:
                    self.speech_audio_buffer = [self.speech_prefetch_buffer]
                self.speech_audio_buffer.append(as_uint8)
            else:
                self.speech_prefetch_buffer = np.roll(self.speech_prefetch_buffer, -len(as_uint8))
                self.speech_prefetch_buffer[-len(as_uint8):] = as_uint8

    def on_timer(self):
        stamp = self.get_clock().now()
        is_voice = self.respeaker.is_voice()
        doa_rad = math.radians(self.respeaker.direction - 180.0)
        doa_rad = angles.shortest_angular_distance(
            doa_rad, math.radians(self.doa_yaw_offset.value))
        doa = math.degrees(doa_rad)

        # vad
        if is_voice != self.prev_is_voice:
            self._pub_vad.publish(Bool(data=is_voice == 1))
            self.prev_is_voice = is_voice

        # doa
        if doa != self.prev_doa:
            self._pub_doa_raw.publish(Int32(data=int(doa)))
            self.prev_doa = doa

            msg = PoseStamped()
            msg.header.frame_id = self.sensor_frame_id.value
            msg.header.stamp = stamp.to_msg()
            ori = T.quaternion_from_euler(math.radians(doa), 0, 0)
            msg.pose.position.x = self.doa_xy_offset.value * np.cos(doa_rad)
            msg.pose.position.y = self.doa_xy_offset.value * np.sin(doa_rad)
            msg.pose.orientation.w = ori[0]
            msg.pose.orientation.x = ori[1]
            msg.pose.orientation.y = ori[2]
            msg.pose.orientation.z = ori[3]
            self._pub_doa.publish(msg)

        # speech audio
        if is_voice:
            self.speech_stopped = stamp
        if stamp - self.speech_stopped < rclpy.duration.Duration(nanoseconds=self.speech_continuation.value * 1e9):
            self.is_speaking = True
        elif self.is_speaking:
            buffered_speech = self.speech_audio_buffer
            self.speech_audio_buffer = []
            self.is_speaking = False
            if len(buffered_speech) == 0:
                return
            buffered_speech = np.hstack(buffered_speech)
            duration = 8 * len(buffered_speech) * self.respeaker_audio.bitwidth
            duration = duration / self.respeaker_audio.rate / self.respeaker_audio.bitdepth
            print("Speech detected for %.3f seconds" % duration)
            if self.speech_min_duration.value <= duration < self.speech_max_duration.value:
                if self._pub_speech_audio.get_subscription_count() > 0:
                    self._pub_speech_audio.publish(AudioData(data=buffered_speech))

    def on_status_led(self, msg):
        self.respeaker.set_led_color(r=msg.r, g=msg.g, b=msg.b, a=msg.a)
        if self.timer_led and self.timer_led.is_alive():
            self.timer_led.destroy()
        self.timer_led = rclpy.timer.Timer(rclpy.duration.Duration(3.0),
                                     lambda e: self.respeaker.set_led_trace(),
                                     oneshot=True)


def main(args=None):
    rclpy.init(args=args)

    publisher = RespeakerNode()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
