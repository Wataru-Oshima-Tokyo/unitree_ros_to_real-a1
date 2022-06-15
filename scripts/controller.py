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
rospy.spin()