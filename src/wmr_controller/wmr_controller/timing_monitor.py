#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Vector3
import time


class TimingMonitor(Node):
    def __init__(self):
        super().__init__('timing_monitor')
        
        # Match the QoS settings
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.subscription = self.create_subscription(
            Vector3,
            '/cmd_unicycle',
            self.callback,
            qos_profile
        )
        
        self.last_time = None
        self.intervals = []
        self.msg_count = 0
        
        # Print summary every 5 seconds
        self.timer = self.create_timer(5.0, self.print_summary)
        
        self.get_logger().info('Timing monitor started. Monitoring /cmd_unicycle...')
    
    def callback(self, msg):
        current_time = time.time()
        
        if self.last_time is not None:
            interval = (current_time - self.last_time) * 1000  # Convert to ms
            self.intervals.append(interval)
        
        self.last_time = current_time
        self.msg_count += 1
    
    def print_summary(self):
        if not self.intervals:
            self.get_logger().warn('No messages received yet...')
            return
        
        avg_interval = sum(self.intervals) / len(self.intervals)
        min_interval = min(self.intervals)
        max_interval = max(self.intervals)
        
        # Calculate standard deviation
        variance = sum((x - avg_interval) ** 2 for x in self.intervals) / len(self.intervals)
        std_dev = variance ** 0.5
        
        avg_hz = 1000.0 / avg_interval if avg_interval > 0 else 0

        # count warnings for intervals deviating more than 10 ms from target 100ms
        warning_count = sum(1 for x in self.intervals if abs(x - 100.0) > 10.0)
        if warning_count > 0:
            self.get_logger().warn(f'Warning: {warning_count} intervals deviated more than 10 ms from target 100 ms')
        
        self.get_logger().info(
            f'\n'
            f'  Messages received: {self.msg_count}\n'
            f'  Average interval: {avg_interval:.2f} ms ({avg_hz:.2f} Hz)\n'
            f'  Min interval: {min_interval:.2f} ms\n'
            f'  Max interval: {max_interval:.2f} ms\n'
            f'  Std deviation: {std_dev:.2f} ms\n'
            f'  Target: 100.00 ms (10.00 Hz)'
        )
        
        # Reset for next window
        self.intervals = []


def main(args=None):
    rclpy.init(args=args)
    node = TimingMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
