#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Bool


class ActionSequenceMonitor(Node):
    def __init__(self):
        super().__init__('action_sequence_monitor')

        self.busy = False
        self.action_count = 0
        self.violation_count = 0
        self.last_action = None

        self.create_subscription(
            Int32,
            '/rl_action',
            self.action_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/rl_action_done',
            self.action_done_callback,
            10
        )

        self.get_logger().info('Action sequence monitor started.')
        self.get_logger().info('Checking /rl_action and /rl_action_done synchronization.')

    def action_callback(self, msg: Int32):
        action = int(msg.data)
        self.action_count += 1

        if self.busy:
            self.violation_count += 1

            self.get_logger().warn(
                f'VIOLATION #{self.violation_count}: '
                f'New action {action} received before previous action finished. '
                f'Last action: {self.last_action}'
            )
        else:
            self.get_logger().info(
                f'Action accepted sequence: {action}'
            )

        self.busy = True
        self.last_action = action

    def action_done_callback(self, msg: Bool):
        done = bool(msg.data)

        if done:
            if self.busy:
                self.get_logger().info(
                    f'Action finished. Last action: {self.last_action}'
                )
            else:
                self.get_logger().info(
                    'Received action_done=True while monitor was not busy.'
                )

            self.busy = False

        else:
            self.get_logger().info('Controller reported action_done=False.')

    def destroy_node(self):
        self.get_logger().info('===============================================')
        self.get_logger().info('Action Sequence Monitor Summary')
        self.get_logger().info('===============================================')
        self.get_logger().info(f'Total actions: {self.action_count}')
        self.get_logger().info(f'Violations: {self.violation_count}')
        self.get_logger().info('===============================================')

        super().destroy_node()


def main():
    rclpy.init()

    node = ActionSequenceMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()