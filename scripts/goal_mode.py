#!/usr/bin/env python3


import rospy
from std_msgs.msg import Int16,Float32,Float64,Int32,Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,PoseStamped

from enum import Enum, IntEnum
from time import time 

#machine states for moving the robot 


#state 01 - > idle  -> stoped motors white
# ----waiting blue 
#state 02 -> with goal -> actived PID yellow
# --- moving---- 
#state 03 -> achieved goal  ->  led green 
# ---- stop 

current_position_x = 0
current_position_y = 0
current_speed = 0

goal_pid_x = None
goal_pid_y = None
goal_pid = None

last_goal_pid = None
reached_goal_flag = False
last_in_goal = False
emergency = False

mission_completed_flag = False

DIST_TOLERANCE = 2 #[m]
SPEED_TOLERANCE = 0.05 #[m/s]
IN_GOAL_TIME = 0 #[s]
IN_GOAL_MAX_TIME = 10 #[s]

cmd_vel_msg = Twist()

# real_goal = 12.4

class Fred_state(IntEnum):
            
    IDLE = 0#white 
    WAITING = 1 #blue
    WITH_GOAL = 50 #yellow
    MOVING_TO_GOAL = 40 #pink
    AT_GOAL =  3 #green
    MISSION_COMPLETED = 6 #orange
    EMERGENCY_BREAK = 2 #red
    




def reached_goal(current_position_x, current_position_y,goal_pid_x,goal_pid_y):
    global reached_goal_flag 
    global reached_goal_time
    global last_in_goal
    # print(f"{goal_pid_x}  {goal_pid_y}")

    if(goal_pid_x == None and  goal_pid_y == None):
        return False
    if(goal_pid_x == None):
        goal_pid_x = 0
    if(goal_pid_y == None):
        goal_pid_y = 0
    current_time = time()
    distance_to_target_x = goal_pid_x - current_position_x
    distance_to_target_y = goal_pid_y - current_position_y
    # print(distance_to_target_x)

    # se a distancia pro objetivo for menor que o objetivo -> esta no objetivo
    in_goal_x = abs(distance_to_target_x) < DIST_TOLERANCE
    in_goal_y = abs(distance_to_target_y) < DIST_TOLERANCE
    
    in_goal = in_goal_x and in_goal_y

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
    

def position_callback(pose_msg):
    global goal_pid_x
    global goal_pid_y
    goal_pid_x =  float(pose_msg.pose.position.x)
    goal_pid_y = float( pose_msg.pose.position.y )
  


def odom_callback(odom_msg):
    global current_position_x
    global current_position_y
    global current_speed

    current_speed = odom_msg.twist.twist.linear.x
    current_position_x = odom_msg.pose.pose.position.x
    current_position_y = odom_msg.pose.pose.position.y

def msg_callback(value, dict):


    if value == 'True' or value == 'true' or value == True:

        dict['value'] = True


    elif value == 'False' or value == 'false' or value == False:

        dict['value'] = False

    else:

        dict['value'] = value



if __name__ == '__main__':
    rospy.init_node('machine_state_node')
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

    mission_completed_dict = {'value': False}   
    mission_completed = mission_completed_dict['value']

    rospy.Subscriber("/goal_manager/goal/mission_completed", Bool, lambda msg: msg_callback(msg.data, mission_completed_dict))


    #PUBS    
   
   # pub_auto_mode = rospy.Publisher('/machine_state/control_mode/auto', Bool, queue_size=1)
    pub_goal_reached = rospy.Publisher("goal_manager/goal/reached",Bool, queue_size = 1)
    pub_cmd_vel = rospy.Publisher("cmd_vel",Twist,queue_size = 10)

    while not rospy.is_shutdown():
        # INPUTS 

        auto_mode = auto_mode_dict['value']
        manual_mode = not auto_mode
        control_conected = control_conection_dict['value'] 
        control_desconected = not control_conected
        abort = abort_dict['value']
        safe = not abort
        moving = fred_moving(current_speed)
        stoped = not moving
        mission_completed = mission_completed_dict['value']
        
        reached_goal_flag = reached_goal(current_position_x, current_position_y,goal_pid_x,goal_pid_y)

        rospy.Subscriber("/odom", Odometry, odom_callback)
        # rospy.Subscriber("/control/position/x", Float64, position_callback)
        # rospy.Subscriber("/control/position/setup/goal", Pose2D, position_callback)
        rospy.Subscriber("/goal_manager/goal/current", PoseStamped, position_callback)


        pub_fita_led = rospy.Publisher("/cmd/led_strip/color",Float32,queue_size = 5)
        pub_turn_on_pid = rospy.Publisher("/control/on",Bool, queue_size = 1)
        
       
        # print(f"state: {state}| auto_mode: {auto_mode}| control_conection: {control_conected}| current pos: {current_position}| goal pid : {goal_pid}|reached_goal_flag {reached_goal_flag }|moving {moving}  ")
        
        if( manual_mode or control_desconected):
            state = Fred_state.IDLE

        if(auto_mode and control_conected):

            if(goal_pid_x == None and stoped): #se ele não tem comando e não esta se movendo 
                state = Fred_state.WAITING

            if(goal_pid_x != None): #tem comando 
                state = Fred_state.WITH_GOAL

                if(moving and not reached_goal_flag): # se esta se movendo e ainda não chegou no objetivo
                    state = Fred_state.MOVING_TO_GOAL

                if(reached_goal_flag ): #chegou no objetivo e não esta se movendo 
                    state = Fred_state.AT_GOAL

                if(mission_completed): #completou todos os objetivos 
                    state = Fred_state.MISSION_COMPLETED

            # if(reached_goal_flag and not moving):
            #         state = Fred_state.STOPING
        if(abort):
            state = Fred_state.EMERGENCY_BREAK
                    

        last_goal_pid_x = goal_pid_x 
        last_goal_pid_y = goal_pid_y  

        #-------------------------------act on state 
        if(state == Fred_state.AT_GOAL):
           goal_pid_x = None
           pub_turn_on_pid.publish(False)

        
        if(state == Fred_state.WITH_GOAL):
            pub_turn_on_pid.publish(True)
        
        if(state == Fred_state.IDLE):
            pub_turn_on_pid.publish(False)


        if(state == Fred_state.MISSION_COMPLETED):
            pub_turn_on_pid.publish(False)
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.angular.z = 0
            pub_cmd_vel.publish(cmd_vel_msg)
            mission_completed_flag = not mission_completed_flag
            if(mission_completed_flag):
                state = 300


        if(state == Fred_state.EMERGENCY_BREAK):
            pub_turn_on_pid.publish(False)
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.angular.z = 0
            pub_cmd_vel.publish(cmd_vel_msg)
            

        if(control_desconected or abort):
            pub_turn_on_pid.publish(False)
            emergency = not emergency
            if(emergency):
                state = 200
            

        mission_completed = False
        pub_goal_reached.publish(reached_goal_flag)
        pub_fita_led.publish(state)
       
            


        rate.sleep()

