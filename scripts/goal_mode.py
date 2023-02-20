#!/usr/bin/env python3


import rospy
from std_msgs.msg import Int16,Float32,Float64,Int32,Bool
from nav_msgs.msg import Odometry

from enum import Enum, IntEnum
from time import time 

#machine states for moving the robot 


#state 01 - > idle  -> stoped motors white
# ----waiting blue 
#state 02 -> with goal -> actived PID yellow
# --- moving---- 
#state 03 -> achieved goal  ->  led green 
# ---- stop 

current_position = 0
current_speed = 0
goal_pid = None
last_goal_pid = None
reached_goal_flag = False
last_in_goal = False
emergency = False

DIST_TOLERANCE = 0.2 #[m]
SPEED_TOLERANCE = 0.1 #[m/s]
IN_GOAL_TIME = 2 #[s]
IN_GOAL_MAX_TIME = 10 #[s]

class Fred_state(IntEnum):
            
    IDLE = 0#white 
    WAITING = 1 #blue
    WITH_GOAL = 5 #yellow
    MOVING_TO_GOAL = 4 #pink
    AT_GOAL =  3 #green
    STOPING = 6 #orange
    EMERGENCY_BREAK = 2 #red



def reached_goal(current_position, goal_pid):
    global reached_goal_flag 
    global reached_goal_time
    global last_in_goal
   
    if(goal_pid==None):
        return False
    current_time = time()
    distance_to_target = goal_pid - current_position

    # se a distancia pro objetivo for menor que o objetivo -> esta no objetivo
    in_goal = abs(distance_to_target) < DIST_TOLERANCE

    # se tiver no objetivo e a flag for falsa -> primeira vez no objetivo guarda o tempo
    #rising edge
    if(in_goal > last_in_goal):
        reached_goal_time = current_time
        
    
    # se nao estiver no objetivo garante q a flag esteja falsa, tbm vale caso ele saia do objetivo
    if(not in_goal):
        reached_goal_time = current_time

    #calcula o tempo que ele ficou no objetivo 
    in_target_time = current_time - reached_goal_time
   
    last_in_goal = in_goal
    # se ele ficou no objetivo pelo tempo necessario retorna verdadeiro

    # print(f"current_position:{current_position}|goal_pid:{goal_pid}|  reached tgt:{in_target_time > IN_GOAL_MAX_TIME} |in_goal:{in_goal}| reached_goal_flag:{reached_goal_flag}|reached_goal_time:{reached_goal_time}|in_target_time:{in_target_time}")
    return in_target_time > IN_GOAL_TIME #and in_target_time < IN_GOAL_MAX_TIME
        

def fred_moving(current_speed):
    return abs(current_speed) > SPEED_TOLERANCE
    

def position_callback(position_msg):
    global goal_pid
    goal_pid = position_msg.data

def odom_callback(odom_msg):
    global current_position
    global current_speed

    current_speed = odom_msg.twist.twist.linear.x
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

    abort_dict = {'value': False}   
    abort = abort_dict['value']

    rospy.Subscriber("/safety/emergency/stop", Bool, lambda msg: msg_callback(msg.data, abort_dict))

    #PUBS    
   
   # pub_auto_mode = rospy.Publisher('/machine_state/control_mode/auto', Bool, queue_size=1)

    while not rospy.is_shutdown():
        # INPUTS 

        auto_mode = auto_mode_dict['value']
        control_conected = control_conection_dict['value'] 
        abort = abort_dict['value']
        safe = not abort
        moving = fred_moving(current_speed)
        reached_goal_flag = reached_goal(current_position,goal_pid)

        rospy.Subscriber("/odom", Odometry, odom_callback)
        rospy.Subscriber("/control/position/x", Float64, position_callback)

        pub_fita_led = rospy.Publisher("/cmd/led_strip/color",Float32,queue_size = 5)
        pub_turn_on_pid = rospy.Publisher("/control/on",Bool, queue_size = 1)
        
       
        # print(f"state: {state}| auto_mode: {auto_mode}| control_conection: {control_conected}| current pos: {current_position}| goal pid : {goal_pid}|reached_goal_flag {reached_goal_flag }|moving {moving}  ")
        
        if( not auto_mode or not control_conected):
            state = Fred_state.IDLE

        if(auto_mode and control_conected):

            if(goal_pid == None and not moving): #se ele não tem comando e não esta se movendo 
                state = Fred_state.WAITING

            if(goal_pid != None): #tem comando 
                state = Fred_state.WITH_GOAL

                if(moving and not reached_goal_flag): # se esta se movendo e ainda não chegou no objetivo
                    state = Fred_state.MOVING_TO_GOAL

                if(reached_goal_flag and not moving): #chegou no objetivo e não esta se movendo 
                    state = Fred_state.AT_GOAL

            # if(reached_goal_flag and not moving):
            #         state = Fred_state.STOPING
        if(abort):
            state = Fred_state.EMERGENCY_BREAK
                    

        last_goal_pid = goal_pid  

        #-------------------------------act on state 
        if(state == Fred_state.AT_GOAL):
           goal_pid = None
           pub_turn_on_pid.publish(False)

        
        if(state == Fred_state.WITH_GOAL):
            pub_turn_on_pid.publish(True)
        
        if(state == Fred_state.IDLE):
            pub_turn_on_pid.publish(False)

        # if(state == Fred_state.EMERGENCY_BREAK):
        #     pub_turn_on_pid.publish(False)

        if( not control_conected or abort):
            pub_turn_on_pid.publish(False)
            emergency = not emergency
            if(emergency):
                state = 200
            

        pub_fita_led.publish(state)
       
            


        rate.sleep()

