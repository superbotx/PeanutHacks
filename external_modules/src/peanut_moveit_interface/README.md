# Peanut Moveit Interface

This is a pure ROS package that is responsible for taking high level command
from the host code, deploying the action to physical or simulator arm using
moveit, and reporting back to the host when necessary.

This code will constantly listen to a ROS service created by the
bridge.

## Service

The service that this interface exposes uses the `peanut_moveit_interface/srv/MoveitInterface.srv` service (which autogenerates ros messages).
