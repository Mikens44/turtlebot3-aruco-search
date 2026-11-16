"""Process lifecycle shared by the three executable nodes."""

import signal
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions


def run_node(factory, args=None, threads=1):
    # ROS launch can forward SIGINT after the terminal has already delivered it
    # to the process group. Repeated signals must not interrupt goal cleanup.
    stopping = threading.Event()
    previous = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    for signum in previous:
        signal.signal(signum, lambda *_: stopping.set())
    node = None
    executor = None
    try:
        node = factory()
        executor = (MultiThreadedExecutor(num_threads=threads) if threads > 1
                    else SingleThreadedExecutor())
        executor.add_node(node)
        while rclpy.ok() and not stopping.is_set():
            executor.spin_once(timeout_sec=.1)
    finally:
        if node is not None and hasattr(node, 'stop'):
            node.stop()
            # Keep action response callbacks running during bounded cancellation.
            limit = time.monotonic() + 2 * node.p['stop_timeout'] + 1.0
            while node.busy and rclpy.ok() and time.monotonic() < limit:
                executor.spin_once(timeout_sec=.05)
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous.items():
            signal.signal(signum, handler)
