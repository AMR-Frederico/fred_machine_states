
#!/usr/bin/env python3

# change between manual control and autonomos mode 


#flage mode -> # fred/states/controler/ -> controler 
#                                       -> auto 




import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from std_msgs.msg import Int32



def msg_callback(value, dict):


    if value == 'True' or value == 'true' or value == True:

        dict['value'] = True


    elif value == 'False' or value == 'false' or value == False:

        dict['value'] = False

    else:

        dict['value'] = value



if __name__ == '__main__':
    rospy.init_node('control_mode_node')
    rate = rospy.Rate(10)
    last_switch_mode = False
    #declare subs   

    #topico que troca de um para o outro
    switch_mode_dict = {'value': False}   
    switch_mode = switch_mode_dict['value']

    rospy.Subscriber("/machine_state/control_mode/switch", Bool, lambda msg: msg_callback(msg.data, switch_mode_dict))

    #topico que acompanha se o modo manual esta ativado
    manual_mode_dict = {'value': False}   
    manual_mode = manual_mode_dict['value']

    rospy.Subscriber("/machine_state/control_mode/manual", Bool, lambda msg: msg_callback(msg.data, manual_mode_dict))

    #topico que acompanha se o modo autonomo esta ativado
    auto_mode_dict = {'value': False}   
    auto_mode = auto_mode_dict['value']

    rospy.Subscriber("/machine_state/control_mode/auto", Bool, lambda msg: msg_callback(msg.data, auto_mode_dict))

    #PUBS 
    pub_manual_mode = rospy.Publisher('/machine_state/control_mode/manual', Bool, queue_size=1)
    pub_auto_mode = rospy.Publisher('/machine_state/control_mode/auto', Bool, queue_size=1)

    while not rospy.is_shutdown():
        switch_mode = switch_mode_dict['value']

        if(switch_mode > last_switch_mode):
                #se mandar um comando positivo troca do modo atual para o outro 
                if(manual_mode and not auto_mode):

                        #desliga manual mode
                        pub_manual_mode.publish(False)
                        # liga auto mode
                        pub_auto_mode.publish(True)
                
                if(auto_mode and not manual_mode):

                        #desliga mannual mode
                        pub_auto_mode.publish(False)
                        #liga manual mode
                        pub_manual_mode.publish(True)

        #Reseta 
        last_switch_mode = switch_mode
        switch_mode_dict['value'] = False 

        rate.sleep()