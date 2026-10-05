import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from pb2025_sentry_decision_interfaces.msg import TacticalExecution
rclpy.init()
n = Node('decision_interface_probe')
events = []
n.create_subscription(TacticalExecution, '/tactical_execution', lambda m: (events.append(m), print(m.decision_id, m.state, m.reason, flush=True)), 10)
p = n.create_client(SetParameters, '/sentry_referee_simulator/set_parameters')
f = n.create_client(SetParameters, '/mock_navigation_interface_only/set_parameters')
def wait(check, seconds=30):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.1)
        if check(): return
    raise AssertionError('Interface test timeout')
def set_value(client, key, value):
    future = client.call_async(SetParameters.Request(parameters=[Parameter(key, value=value).to_parameter_msg()]))
    wait(future.done)
    assert future.result().results[0].successful
wait(lambda:p.service_is_ready() and f.service_is_ready())
wait(lambda:any(e.state=='STARTED' and e.decision_id for e in events))
set_value(p, 'scenario', 'hold_safe')
wait(lambda:any(e.state=='PREEMPTED' for e in events))
set_value(p, 'scenario', 'gather_info')
wait(lambda:any(e.state=='SUCCEEDED' for e in events))
set_value(p, 'scenario', 'hold_safe')
set_value(f, 'outcome', 'abort')
set_value(p, 'scenario', 'gather_info')
wait(lambda:any(e.state=='FAILED' for e in events))
set_value(p, 'scenario', 'hold_safe')
print('PASS: mock interface goal/cancel/success/failure; not physics acceptance', flush=True)
n.destroy_node()
rclpy.shutdown()
