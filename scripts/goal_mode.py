#!/usr/bin/env python3


import rospy
from std_msgs.msg import Int16,Float32,Float64,Int32,Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,PoseStamped

from enum import Enum, IntEnum
from time import time 



current_position_x = 0
current_position_y = 0
current_speed = 0


goal_pid_x = None
goal_pid_y = None
goal_pid = None
ghost_goal = None

ghost_goal_flag = False 

last_goal_pid = None
reached_goal_flag = False
last_in_goal = False
emergency = False

mission_completed_flag = False

SPEED_TOLERANCE = 0.05 #[m/s]

cmd_vel_msg = Twist()

state_msg = Int16()

class Fred_state(IntEnum):
            
    MANUAL = 0#white 
    WAITING = 1 #blue
    AUTONOMO = 50 #yellow
    AT_GOAL =  3 #green
    MISSION_COMPLETED = 6 #orange
    EMERGENCY = 2 #red
    


def goal_reached_callback( data):
     global reached_goal_flag
     reached_goal_flag = data.data



def msg_callback(value, dict):


    if value == 'True' or value == 'true' or value == True:

        dict['value'] = True


    elif value == 'False' or value == 'false' or value == False:

        dict['value'] = False

    else:

        dict['value'] = value



if __name__ == '__main__':
    rospy.init_node('machine_state_node')
    rate = rospy.Rate(50)

    state = Fred_state.MANUAL
   
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

    mission_completed_dict = {'value': False}   
    mission_completed = mission_completed_dict['value']

    rospy.Subscriber("/goal_manager/goal/mission_completed", Bool, lambda msg: msg_callback(msg.data, mission_completed_dict))

    rospy.Subscriber("/goal_manager/goal/spline/reached", Bool,goal_reached_callback) 


    #PUBS    
   

    pub_cmd_vel = rospy.Publisher("cmd_vel",Twist,queue_size = 10)


    pub_main_machine_state = rospy.Publisher("/machine_state/main",Int16,queue_size = 5)

    pub_turn_on_pid = rospy.Publisher("/navigation/on",Bool, queue_size = 1)

    while not rospy.is_shutdown():
        # INPUTS 

        auto_mode = auto_mode_dict['value']
        manual_mode = not auto_mode

        control_conected = control_conection_dict['value'] 
        control_desconected = not control_conected

        abort = abort_dict['value']
        safe = not abort and control_conected

     

    

        mission_completed = mission_completed_dict['value']
        
        
         rospy.loginfo(f"MAIN MACHINE STATES:  state: {state}| auto_mode: {auto_mode}| control_conection: {control_conected}| current pos: {current_position}| goal pid : {goal_pid}|reached_goal_flag {reached_goal_flag }|moving {moving}  ")
        
        if( manual_mode and safe):
            state = Fred_state.MANUAL
           

        if(auto_mode and safe):

            state = Fred_state.AUTONOMO

            if(mission_completed): #completou todos os objetivos 
                    state = Fred_state.MISSION_COMPLETED

           
        if(abort):
            state = Fred_state.EMERGENCY
            
                    

        last_goal_pid_x = goal_pid_x 
        last_goal_pid_y = goal_pid_y  

        state_msg.data = state
        # print(state)
        pub_main_machine_state.publish(state_msg)
        #-------------------------------act on state 
   
        
        if(state == Fred_state.AUTONOMO):
            pub_turn_on_pid.publish(True)
        
        if(state == Fred_state.MANUAL):
            pub_turn_on_pid.publish(False)


        if(state == Fred_state.MISSION_COMPLETED):
            pub_turn_on_pid.publish(False)
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.angular.z = 0
            pub_cmd_vel.publish(cmd_vel_msg)
           


        if(state == Fred_state.EMERGENCY):
            pub_turn_on_pid.publish(False)
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.angular.z = 0
            pub_cmd_vel.publish(cmd_vel_msg)
            

      
            

        mission_completed = False
        

       
        rate.sleep()

