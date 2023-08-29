# respeaker_ros

ROS2 wrapper for the ReSpeaker 4 Mic Array. Publishes audio, direction-of-arrival information and allows control over the LED ring.

Based on [previous](https://github.com/furushchev/respeaker_ros) [wrappers](https://github.com/machinekoder/respeaker).

## Usage

You need to build [audio_common](https://github.com/ros-drivers/audio_common) from source until it gets released into ROS2.

Copy the udev rules from `respeaker_ros/config` into `/etc/udev/rules.d/`.

