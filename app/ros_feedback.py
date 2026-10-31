"""Process a bounded batch so scene subscriptions cannot starve each other."""
import time


def spin_some(node, timeout_sec=0., max_callbacks=64, budget=.01):
    import rclpy
    rclpy.spin_once(node, timeout_sec=timeout_sec)
    deadline=time.monotonic()+budget
    for _ in range(max_callbacks-1):
        if time.monotonic()>=deadline:
            break
        rclpy.spin_once(node, timeout_sec=0.)
