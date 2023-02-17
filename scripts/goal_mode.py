#!/usr/bin/env python3


import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Int32

#machine states for moving the robot 


#state 01 - > idle  -> stoped motors led yellow 
# ----start moving  
#state 02 -> with goal -> actived PID led blue 
# --- moving---- 
#state 03 -> achieved goal  ->  led green 
# ---- stop 



