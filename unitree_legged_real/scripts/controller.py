#!/usr/bin/env python
import subprocess
import os
import signal
import psutil
import time

import rospy
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

import sys

rospy.init_node('process_manager')
subprocess.Popen('gnome-terminal -- bash -c "cd $HOME/catkin_ws; source /opt/ros/melodic/setup.bash; source $HOME/catkin_ws/devel/setup.bash; rosrun teleop_twist_keyboard teleop_twist_keyboard.py __name:=move_controller _key_timeout:=0.6 _speed:=0.2 _turn:=0.2 cmd_vel:=/cmd_vel_move; exec bash"', shell=True)
time.sleep(3)
subprocess.Popen('gnome-terminal -- bash -c "cd $HOME/catkin_ws; source /opt/ros/melodic/setup.bash; source $HOME/catkin_ws/devel/setup.bash; rosrun teleop_twist_keyboard teleop_twist_keyboard.py __name:=posture_controller _key_timeout:=0.6 _speed:=0.2 _turn:=0.2 cmd_vel:=/cmd_vel_posture; exec bash"', shell=True)

rospy.spin()
