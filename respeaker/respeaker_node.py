#!/usr/bin/env python3
import usb.core
import usb.util

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16

from .firmware.tuning import Tuning


class RespeakerNode(Node):
    def __init__(self):
        super().__init__('respeaker_node')

        self.declare_parameter('update_period_s', 0.1)

        dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        self._tuning = Tuning(dev)

        update_period_s = self.get_parameter('update_period_s').value
        self.get_logger().info(
            'Starting with update period {}'.format(update_period_s)
        )
        self._pub = self.create_publisher(UInt16, 'respeaker/doa', 10)
        self._timer = self.create_timer(update_period_s, self.timer_callback)

    def timer_callback(self):
        msg = UInt16()
        msg.data = self._tuning.direction
        self._pub.publish(msg)
        self.get_logger().debug('Publishing DOA {} '.format(msg.data))


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
