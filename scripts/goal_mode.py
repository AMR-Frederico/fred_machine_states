#!/usr/bin/env python3


import rospy
from std_msgs.msg import Int16,Float32,Float64,Int32,Bool
from nav_msgs.msg import Odometry

from enum import Enum, IntEnum
#machine states for moving the robot 


#state 01 - > idle  -> stoped motors led yellow 
# ----start moving  
#state 02 -> with goal -> actived PID led blue 
# --- moving---- 
#state 03 -> achieved goal  ->  led green 
# ---- stop 

current_position = None
goal_pid = None

class Fred_state(IntEnum):

    IDLE = 0
    WAITING = 100
    WITH_GOAL = 110
    MOVING_TO_GOAL = 120
    AT_GOAL = 130 # added by Ale
    STOPING = 200
    

def position_callback(position_msg):
    global goal_pid
    goal_pid = position_msg.data

def odom_callback(odom_msg):
    global current_position
    current_position = odom_msg.pose.pose.position.x

def msg_callback(value, dict):


    if value == 'True' or value == 'true' or value == True:

        dict['value'] = True


    elif value == 'False' or value == 'false' or value == False:

        dict['value'] = False

    else:

        dict['value'] = value



if __name__ == '__main__':
    rospy.init_node('goal_manager_node')
    rate = rospy.Rate(10)

    state = Fred_state.IDLE
    last_switch_mode = False
    #declare subs   

    
   
    #topico que acompanha se o modo autonomo esta ativado
    auto_mode_dict = {'value': False}   
    auto_mode = auto_mode_dict['value']

    rospy.Subscriber("/machine_state/control_mode/auto", Bool, lambda msg: msg_callback(msg.data, auto_mode_dict))

    #SAFE topico que verifica q tem conexão com o controle 
    control_conection_dict = {'value': False}   
    control_conection = control_conection_dict['value']

    rospy.Subscriber("joy/controler/connected", Bool, lambda msg: msg_callback(msg.data, control_conection_dict))
    #PUBS    
   
   # pub_auto_mode = rospy.Publisher('/machine_state/control_mode/auto', Bool, queue_size=1)

    while not rospy.is_shutdown():
        # INPUTS 

        auto_mode = auto_mode_dict['value']
        control_conected = control_conection_dict['value'] 
        moving = True 
        reached_goal = True

        rospy.Subscriber("/odom", Odometry, odom_callback)
        rospy.Subscriber("/control/position/x", Float64, position_callback)

        print(f"state: {state}| auto_mode: {auto_mode}| control_conection: {control_conected}| current pos: {current_position}| goal pid : {goal_pid}")
        
        if( not auto_mode or not control_conected):
            state = Fred_state.IDLE

        if(auto_mode and control_conected):

            if(goal_pid == None): #se ele não tem comando e não esta se movendo 
                state = Fred_state.WAITING

            if(goal_pid != None): #tem comando 
                state = Fred_state.WITH_GOAL

                if(moving and not reached_goal): # se esta se movendo e ainda não chegou no objetivo
                    state = Fred_state.MOVING_TO_GOAL

                if(reached_goal and not moving): #chegou no objetivo e não esta se movendo 
                    state = Fred_state.AT_GOAL

            


        rate.sleep()

