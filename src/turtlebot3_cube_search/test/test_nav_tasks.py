from concurrent.futures import Future
from types import SimpleNamespace
import time

from turtlebot3_cube_search.nav_tasks import ActionTask


class FakeHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result = Future()
        self.cancel_count = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancel_count += 1
        future = Future()
        future.set_result(object())
        return future


class FakeClient:
    def __init__(self):
        self.sent = Future()

    def send_goal_async(self, goal):
        return self.sent


def test_cancel_before_acceptance_still_cancels_late_goal():
    client = FakeClient()
    task = ActionTask(client, object())
    assert not task.cancel_and_wait(.001)
    handle = FakeHandle()
    client.sent.set_result(handle)
    assert handle.cancel_count == 1
    # Cancel response alone must not be treated as a terminal action result.
    assert not task.done.is_set()
    task.cancel()
    assert handle.cancel_count == 1
    handle.result.set_result(SimpleNamespace(status=5, result=object()))
    assert task.cancel_and_wait(.01)
    assert not task.succeeded()


def test_rejection_success_and_failed_result():
    client = FakeClient()
    task = ActionTask(client, object())
    client.sent.set_result(FakeHandle(accepted=False))
    assert task.done.is_set() and not task.succeeded()
    for status, error_code, expected in [(4, 0, True), (4, 1, False), (6, 0, False)]:
        client = FakeClient()
        task = ActionTask(client, object())
        handle = FakeHandle()
        client.sent.set_result(handle)
        handle.result.set_result(SimpleNamespace(
            status=status, result=SimpleNamespace(error_code=error_code)))
        assert task.succeeded() == expected
        assert task.cancel_and_wait(.01)
        assert handle.cancel_count == 0


def test_wait_interrupt_and_timeout():
    task = ActionTask(FakeClient(), object())
    assert task.wait(lambda: True, time.monotonic() + 1) == 'interrupted'
    assert task.wait(lambda: False, time.monotonic() - 1) == 'timeout'


def test_transport_error_does_not_prove_remote_goal_stopped():
    client = FakeClient()
    task = ActionTask(client, object())
    client.sent.set_exception(RuntimeError('Transport failure'))
    assert not task.cancel_and_wait(.001)
    assert isinstance(task.error, RuntimeError)
