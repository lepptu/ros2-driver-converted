#!/usr/bin/env python3
"""Hoverboard power-on orchestration (DEPLOY_TODO 3.1).

At stack startup: if the hoverboard board is not streaming (no `hoverboard/connected`)
after a grace period, press its power button once via the relay (`hoverBtnR1_pulse`)
and wait. Limited retries with backoff.

Deliberate-off protection: the pulse is a TOGGLE. This node only ever acts during
its startup window and stops permanently once the board has been seen connected —
it will never re-pulse a board that someone turned off later.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool


class HoverboardPowerManager(Node):
    def __init__(self):
        super().__init__('hoverboard_power_manager')
        self.declare_parameter('auto_power_on', True)
        self.declare_parameter('grace_period', 20.0)    # s to wait for connected before first pulse
        self.declare_parameter('pulse_wait', 30.0)      # s to wait after a pulse before retrying
        self.declare_parameter('max_attempts', 3)

        self.auto_power_on = self.get_parameter('auto_power_on').value
        self.grace_period = float(self.get_parameter('grace_period').value)
        self.pulse_wait = float(self.get_parameter('pulse_wait').value)
        self.max_attempts = int(self.get_parameter('max_attempts').value)

        # Driver publishes `connected` latched (transient_local): subscribe with
        # matching durability so we receive the current state, not just changes.
        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.connected = False
        self.ever_connected = False
        self.attempts = 0
        self.done = False

        self.sub = self.create_subscription(Bool, 'hoverboard/connected',
                                            self._on_connected, latched)
        self.pulse_pub = self.create_publisher(Bool, 'hoverBtnR1_pulse', 10)

        if not self.auto_power_on:
            self.get_logger().info('auto_power_on disabled - standing by')
            self.done = True
        else:
            self.get_logger().info(
                f'waiting up to {self.grace_period:.0f}s for hoverboard before power-on attempt')
            self.timer = self.create_timer(self.grace_period, self._tick)

    def _on_connected(self, msg: Bool):
        self.connected = bool(msg.data)
        if self.connected and not self.ever_connected:
            self.ever_connected = True
            if not self.done:
                self.get_logger().info('hoverboard connected - power-on orchestration done')
                self.done = True

    def _tick(self):
        if self.done:
            self.timer.cancel()
            return
        if self.connected:
            self.done = True
            self.timer.cancel()
            return
        if self.attempts >= self.max_attempts:
            self.get_logger().error(
                f'hoverboard still not connected after {self.attempts} power-on attempts - '
                'giving up (board unplugged, battery empty, or turned off deliberately?). '
                'Manual: press the power button or publish hoverBtnR1_pulse once.')
            self.done = True
            self.timer.cancel()
            return
        self.attempts += 1
        self.get_logger().info(
            f'hoverboard not connected - relay power-on pulse (attempt {self.attempts}/{self.max_attempts})')
        self.pulse_pub.publish(Bool(data=True))
        # reschedule next check after pulse_wait
        self.timer.cancel()
        self.timer = self.create_timer(self.pulse_wait, self._tick)


def main():
    rclpy.init()
    node = HoverboardPowerManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
