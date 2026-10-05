uint8 STARTED=0
uint8 RUNNING=1
uint8 SUCCEEDED=2
uint8 FAILED=3
uint8 PREEMPTED=4
uint8 STALE=5

std_msgs/Header header
uint32 decision_id
uint32 world_state_id
uint8 action
string state
string reason
