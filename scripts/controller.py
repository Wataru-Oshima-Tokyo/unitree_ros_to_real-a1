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
subprocess.run('shell=True)
               # rosrun teleop_twist_keyboard teleop_twist_keyboard.py _key_timeout:=0.6 _speed:=0.2 _turn:=0.2 cmd_vel:=/cmd_vel_raw
rospy.spin()
