#!/usr/bin/env python
import time

real_init=time.time()
import math
import rospy
import numpy as np



# import Float64 msg
from std_msgs.msg import Float64
# import control msg
from control_msgs.msg import JointControllerState
from std_srvs.srv import Empty

import dynamic_reconfigure.client

print('inicializar funciones')
t_init=time.time()
reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
pause_simulation = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
unpause_simulation = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
print(time.time()-t_init)

data=[]
tiempo=[]
estoyen0=False


def codo_callback(msg):
    #print("  angle: %f " % msg.process_value)
    #print("    set_point: %f " % msg.set_point)

    data.append(msg.error)
    #print(type(msg.error))
    tiempo.append(msg.header.stamp.to_sec())
    #print(type(msg.header.stamp.to_sec()))
    # here we can do more stuff
    # blabla

def main_loop():
    rospy.init_node('run_simulation_once')
    print(rospy.get_time())
    condition= False
    t_escalon=0.5
    t_it=2.0
    # Create publisher for codo

    client = dynamic_reconfigure.client.Client('/nelen/hombro_control/pid')

    pub_codo_cmd = rospy.Publisher('/nelen/codo_cont/command', Float64, queue_size=10)
    sub_codo_state = rospy.Subscriber('/nelen/codo_cont/state', JointControllerState, codo_callback)
    # Set publication rate
    rate = rospy.Rate(100) # 100hz
    pub_codo_cmd.publish(0.0)

    print('tiempo reset')
    t_init=time.time()
    reset_simulation()
    print(time.time()-t_init)
    # Create a subscriber for codo

    print('tiempo despausa')
    t_init=time.time()
    unpause_simulation()
    print(time.time()-t_init)


    # Main loop
    # We'll publish commands at the rate defined before



    t_init=rospy.get_time()
    while not condition:
        t = rospy.get_time() # get current time


        # publish command
        if t>=t_it+t_init:
            pause_simulation()
            print(len(tiempo))
            print(len(data))
            condition = True
            pub_codo_cmd.publish(0.0)
            params={'p':80.0}
            config = client.update_configuration(params)
            print(rospy.get_time()-t_init)
            print('finalmente')
            print(real_init-time.time())

            break

        elif t>=t_escalon+t_init:
            pub_codo_cmd.publish(1.0)

        else:
            pub_codo_cmd.publish(0.0)


        # micro pause to reach the desired rate
        rate.sleep()

    # Check if we have received messages
    # If we had, the callback will be called and the state of the codo will be printed
    #rospy.spin()

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass
