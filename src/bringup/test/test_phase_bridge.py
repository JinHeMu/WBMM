"""Phase request retries and latest-request ordering, without a running ROS graph."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

_spec = importlib.util.spec_from_file_location(
    'phase_bridge', Path(__file__).resolve().parents[1] / 'scripts/remani_phase_bridge.py')
module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(module)


def bridge():
    # Exercise the policy using a lightweight object; Node initialization is not
    # necessary for these service-future state transitions.
    obj = SimpleNamespace(
        client=Mock(), _phase_generation=0, _desired_phase=None,
        _phase_future=None, _execution_requested=False, execution_phase=2,
        get_logger=lambda: Mock())
    for method in ('request_phase', 'retry_phase', 'on_phase_response'):
        setattr(obj, method, getattr(module.RemaniPhaseBridge, method).__get__(obj))
    return obj


def test_request_retries_when_service_becomes_ready():
    obj = bridge()
    obj.client.service_is_ready.return_value = False
    obj.request_phase(2, 'EE target')
    obj.client.call_async.assert_not_called()
    obj.client.service_is_ready.return_value = True
    obj.retry_phase()
    assert obj.client.call_async.call_args.args[0].phase == 2
    assert obj._desired_phase is not None


def test_new_navigation_request_supersedes_inflight_execution_response():
    obj = bridge()
    obj.client.service_is_ready.return_value = True
    old = Mock()
    obj.client.call_async.return_value = old
    obj.request_phase(2, 'EE target')
    obj.request_phase(0, 'new goal')
    obj.on_phase_response(old, 2, 'EE target', 1)
    assert obj._desired_phase[0] == 0
    obj.retry_phase()
    assert obj.client.call_async.call_args.args[0].phase == 0


def test_rejection_is_retried_until_acknowledged():
    obj = bridge()
    obj.client.service_is_ready.return_value = True
    obj.request_phase(0, 'goal')
    response = Mock()
    response.result.return_value = SimpleNamespace(success=False, message='busy')
    obj.on_phase_response(response, 0, 'goal', 1)
    assert obj._desired_phase is not None
    obj.retry_phase()
    response.result.return_value = SimpleNamespace(success=True, requested_phase=0, active_phase=0)
    obj.on_phase_response(response, 0, 'goal', 1)
    assert obj._desired_phase is None
