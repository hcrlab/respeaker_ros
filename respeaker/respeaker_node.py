#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from std_msgs.msg import Header


class RespeakerNode(Node):
    def __init__(self):
        super().__init__('respeaker_node')

        self.declare_parameter('update_period_s', 0.1)

        update_period_s = self.get_parameter('update_period_s').value
        self.get_logger().info(
            'Starting heartbeat with period {}'.format(update_period_s)
        )
        self._pub = self.create_publisher(Header, 'heartbeat', 1)
        self._clock = Clock()
        self._timer = self.create_timer(update_period_s, self.timer_callback)

    def timer_callback(self):
        timestamp = self._clock.now()
        msg = Header()
        msg.stamp = timestamp.to_msg()
        self._pub.publish(msg)
        self.get_logger().debug('Publishing: "%s"' % msg.stamp)


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
