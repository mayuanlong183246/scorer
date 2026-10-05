#!/usr/bin/env python3
"""Live acceptance probe; run in the same ROS_DOMAIN_ID as a fresh autonomous launch.

Changes referee scenarios. Verifies movement, cancellation, stop, recovery and
terminal RViz marker content. Intended for an isolated simulation only.
"""
import time, math
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from pb2025_sentry_decision_interfaces.msg import TacticalExecution
rclpy.init(); n=Node('gazebo_navigation_probe'); p=n.create_client(SetParameters, '/sentry_referee_simulator/set_parameters')
state={'odom':None,'events':[],'markers':None}
def event(m):
 state['events'].append(m); print('EXECUTION',m.decision_id,m.world_state_id,m.action,m.state,m.reason,flush=True)
n.create_subscription(Odometry,'/odom',lambda m:state.update(odom=m),10)
n.create_subscription(TacticalExecution,'/tactical_execution',event,10)
n.create_subscription(MarkerArray,'/tactical_markers',lambda m:state.update(markers=m),10)
def wait(pred,seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.1)
  if pred(): return
 raise RuntimeError('timeout waiting for test condition')
def scenario(s):
 f=p.call_async(SetParameters.Request(parameters=[Parameter('scenario',value=s).to_parameter_msg()])); wait(f.done,5); print('SCENARIO',s,f.result(),flush=True)
wait(lambda:p.service_is_ready(),20)
scenario('gather_info')
wait(lambda:state['odom'] is not None and any(e.state=='STARTED' and e.decision_id for e in state['events']),35)
wait(lambda:math.hypot(state['odom'].pose.pose.position.x-3.4,state['odom'].pose.pose.position.y-9.5)>.6,20)
print('MOVING',state['odom'].pose.pose.position,flush=True)
scenario('hold_safe'); wait(lambda:any(e.state=='PREEMPTED' for e in state['events']),10)
wait(lambda:math.hypot(state['odom'].twist.twist.linear.x,state['odom'].twist.twist.linear.y)<.03,10)
print('STOPPED',state['odom'].twist.twist,flush=True)
scenario('gather_info'); wait(lambda:any(e.action==6 and e.state=='SUCCEEDED' for e in state['events']),45)
print('ARRIVED',state['odom'].pose.pose.position,flush=True)
wait(lambda:any('LAST TASK' in m.text and 'SUCCEEDED' in m.text for m in state['markers'].markers),5)
print('MARKERS',[(m.ns,m.text,m.pose.position.x,m.pose.position.y) for m in state['markers'].markers],flush=True)
scenario('hold_safe')
n.destroy_node();rclpy.shutdown()
