# Peanut Moveit Interface

This is a pure ROS package that is responsible for taking high level command
from the host code, deploying the action to physical or simulator arm using
moveit and action service server, and reporting back to the host when necessary.

This code will constantly listen to a ROS publisher or service created by the
bridge. Upon receiving a new command, it create a new thread or process or
or non-blocking call (developer's preference) to execute the action.

## Minimal example

```python
import arm_srv
import rospy
from util import *

curr_srv = None
group = moveit_commander.MoveGroupCommander("arm")

def handler(req):
  if curr_srv:
    send_terminate_response(curr_srv)
  setup_group(group, req)
  group.go(wait=False)
  group.onFinish(callback=send_finish_response)

if __name__ == "__main__":
  rospy.init_node('moveit_interface')
  s = rospy.Service('moveit_interface', arm_srv, handler)
  rospy.spin()
```
