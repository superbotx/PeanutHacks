# Peanut Moveit Interface

This is a pure ROS package that is responsible for taking high level command
from the host code, deploying the action to physical or simulator arm using
moveit and action service server, and reporting back to the host when necessary.

This code will constantly listen to a ROS publisher or service created by the
bridge. Upon receiving a new command, it create a new thread or process or
or non-blocking call (developer's preference) to execute the action.
