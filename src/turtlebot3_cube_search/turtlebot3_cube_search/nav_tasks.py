"""Track an asynchronous action, including cancellation before goal acceptance.

This module uses the action client's public Future API and has no ROS imports,
so the cancellation races can be tested with deterministic fake clients.
"""

import threading
import time


class ActionTask:
    def __init__(self, client, goal):
        self._lock = threading.RLock()
        self.done = threading.Event()
        self.handle = None
        self.response = None
        self.error = None
        self.cancel_requested = False
        self.cancel_sent = False
        self.cancel_future = None
        try:
            self.send_future = client.send_goal_async(goal)
            self.send_future.add_done_callback(self._on_accepted)
        except Exception as error:
            self.error = error
            # A transport exception does not establish whether the remote goal
            # was accepted. Do not claim a terminal state without its result.

    def _on_accepted(self, future):
        with self._lock:
            try:
                self.handle = future.result()
                if not self.handle.accepted:
                    self.done.set()
                    return
                self.result_future = self.handle.get_result_async()
                self.result_future.add_done_callback(self._on_result)
                # Cancellation may have arrived while send_goal was in flight.
                if self.cancel_requested:
                    self._send_cancel()
            except Exception as error:
                self.error = error
                # If a goal was accepted, a failed result request does not prove
                # that the robot stopped. Leave done unset and request cancel.
                if self.handle is not None and not self.handle.accepted:
                    self.done.set()
                else:
                    self.cancel_requested = True
                    self._send_cancel()

    def _on_result(self, future):
        with self._lock:
            try:
                self.response = future.result()
                self.done.set()
            except Exception as error:
                self.error = error  # Unknown terminal state; fail closed.

    def _send_cancel(self):
        if self.handle is not None and self.handle.accepted and not self.cancel_sent:
            try:
                self.cancel_future = self.handle.cancel_goal_async()
                self.cancel_sent = True
            except Exception as error:
                self.error = error

    def cancel(self):
        with self._lock:
            self.cancel_requested = True
            if not self.done.is_set():
                self._send_cancel()

    def cancel_and_wait(self, timeout):
        self.cancel()
        # Cancel acknowledgement alone is insufficient: wait for terminal result.
        return self.done.wait(timeout)

    def wait(self, should_stop, deadline):
        while not self.done.is_set():
            if should_stop():
                return 'interrupted'
            if time.monotonic() >= deadline:
                return 'timeout'
            self.done.wait(0.02)
        return 'finished'

    def succeeded(self):
        # action_msgs/GoalStatus.STATUS_SUCCEEDED == 4.
        return (self.error is None and self.response is not None and
                self.response.status == 4 and
                getattr(self.response.result, 'error_code', 0) == 0)
