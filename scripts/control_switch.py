#!/usr/bin/env python3

# change between manual control and autonomos mode 


#flage mode -> # fred/states/controler/ -> controler 
#                                       -> auto 




import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from std_msgs.msg import Int32
# from geometry_msgs.msg import Twist

# cmd_vel_manual = Twist()
# cmd_vel_auto = Twist()


# def call_back_auto_controler(msg):
#     global cmd_vel_auto
#     cmd_vel_auto = msg 

# def call_back__manual_controler(msg):
#     global cmd_vel_manual
#     cmd_vel_manual = msg

def msg_callback(value, dict):


    if value == 'True' or value == 'true' or value == True:

        dict['value'] = True


    elif value == 'False' or value == 'false' or value == False:

        dict['value'] = False

    else:

        dict['value'] = value



if __name__ == '__main__':
    rospy.init_node('control_mode_node')
    rate = rospy.Rate(50)
    last_switch_mode = False
    #declare subs   

    #topico que troca de um para o outro
    switch_mode_dict = {'value': False}   
    switch_mode = switch_mode_dict['value']

    rospy.Subscriber("/machine_state/control_mode/switch", Bool, lambda msg: msg_callback(msg.data, switch_mode_dict))

    #topico que acompanha se o modo manual esta ativado
    manual_mode_dict = {'value': True}   
    manual_mode = manual_mode_dict['value']

    rospy.Subscriber("/machine_state/control_mode/manual", Bool, lambda msg: msg_callback(msg.data, manual_mode_dict))

    #topico que acompanha se o modo autonomo esta ativado
    auto_mode_dict = {'value': False}   
    auto_mode = auto_mode_dict['value']

    rospy.Subscriber("/machine_state/control_mode/auto", Bool, lambda msg: msg_callback(msg.data, auto_mode_dict))


    # rospy.Subscriber("cmd_vel/auto_controler",Twist, call_back_auto_controler)
    # rospy.Subscriber("cmd_vel/manual_controler",Twist, call_back__manual_controler)

    #PUBS 
    pub_manual_mode = rospy.Publisher('/machine_state/control_mode/manual', Bool, queue_size=1)
    pub_auto_mode = rospy.Publisher('/machine_state/control_mode/auto', Bool, queue_size=1)
    # pub_cmd_vel = rospy.Publisher("cmd_vel", Twist , queue_size=10)


    while not rospy.is_shutdown():
        switch_mode = switch_mode_dict['value']
        auto_mode = auto_mode_dict['value']
        manual_mode = manual_mode_dict['value']

        if(switch_mode > last_switch_mode):
                #se mandar um comando positivo troca do modo atual para o outro 
                if(manual_mode and not auto_mode):

                        #desliga manual mode
                        pub_manual_mode.publish(False)
                        # liga auto mode
                        pub_auto_mode.publish(True)
                        # pub_cmd_vel.publish(cmd_vel_auto)

                
                if(auto_mode and not manual_mode):

                        #desliga mannual mode
                        pub_auto_mode.publish(False)
                        #liga manual mode
                        pub_manual_mode.publish(True)
                        # pub_cmd_vel.publish(cmd_vel_manual)

                # if(auto_mode and  manual_mode):

                #         #desliga mannual mode
                #         pub_auto_mode.publish(False)
                #         #liga manual mode
                #         pub_manual_mode.publish(True)

                # if( not auto_mode and  not manual_mode):

                #         #desliga mannual mode
                #         pub_auto_mode.publish(False)
                #         #liga manual mode
                #         pub_manual_mode.publish(True)

        #Reseta 
        last_switch_mode = switch_mode
        # print(f"manual_mode: {manual_mode} | auto mode: {auto_mode} | switch_mode: {switch_mode}" )
        switch_mode_dict['value'] = False 

        rate.sleep()