import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class ClockFilter(Node):

    def __init__(self):
        super().__init__('clock_filter')

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.last_stamp_ns = None
        self.dropped = 0
        self.reset_threshold_ns = 1_000_000_000

        self.publisher = self.create_publisher(Clock, '/clock', qos)
        self.subscription = self.create_subscription(
            Clock, '/clock_raw', self.on_clock, qos)

        self.get_logger().info(
            'Republishing /clock_raw as monotonic /clock')

    def on_clock(self, msg):
        stamp_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            backward_ns = self.last_stamp_ns - stamp_ns
            if backward_ns > self.reset_threshold_ns:
                self.get_logger().warning(
                    f'Clock jumped back {backward_ns / 1e9:.3f} s; '
                    'treating it as a simulation reset')
                self.last_stamp_ns = stamp_ns
                self.publisher.publish(msg)
                return

            self.dropped += 1
            if self.dropped % 1000 == 1:
                self.get_logger().warning(
                    f'Dropped {self.dropped} non-monotonic /clock message(s); '
                    f'last regression {backward_ns / 1e9:.6f} s')
            return

        self.last_stamp_ns = stamp_ns
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ClockFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
