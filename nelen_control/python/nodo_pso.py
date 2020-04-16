#!/usr/bin/env python
import time
# Se importan paquetes importantes
import math
import rospy
import numpy as np

# import Float64 msg
from std_msgs.msg import Float64
# import control msg
from control_msgs.msg import JointControllerState

#necesario para llamar a los servicions de pausa despausa y reinicializacion
from std_srvs.srv import Empty

#paquete para cambiar los parametros del PID
import dynamic_reconfigure.client

#se crean funciones para operar gazebo
reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
pause_simulation = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
unpause_simulation = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

#variables universales para guardar los datos de la realziacion
data=[]
tiempo=[]


#esta funcion se opera cada vez que el nodo recibe un mensaje

def listener_callback(msg):
    #print('guardardato')

    #se guarda el error
    data.append(msg.error)
    #se guarda el tiempo
    tiempo.append(msg.header.stamp.to_sec())

def run_once(p,i,d,joint):
    T_abs=time.time()
    rospy.init_node('run_simulation_once')

    condition= False
    t_escalon=0.5
    t_it=2.0
    # suscribirse a los joints segun sea el caso
    if joint=='codo':
        #setear codo
        client = dynamic_reconfigure.client.Client('/nelen/codo_cont/pid')
        pub_codo_cmd = rospy.Publisher('/nelen/codo_cont/command', Float64, queue_size=10)
        sub_codo_state = rospy.Subscriber('/nelen/codo_cont/state', JointControllerState, listener_callback)

    elif joint=='hombro':
        #setear hombro
        client = dynamic_reconfigure.client.Client('/nelen/hombro_control/pid')
        pub_codo_cmd = rospy.Publisher('/nelen/hombro_control/command', Float64, queue_size=10)
        sub_codo_state = rospy.Subscriber('/nelen/hombro_control/state', JointControllerState, listener_callback)

    elif joint=='z':
        #setear z
        client = dynamic_reconfigure.client.Client('/nelen/prism_control/pid')
        pub_codo_cmd = rospy.Publisher('/nelen/prism_control/command', Float64, queue_size=10)
        sub_codo_state = rospy.Subscriber('/nelen/prism_control/state', JointControllerState, listener_callback)

    else:
        print('not a joint')

    params={'p':p,'i':i,'d':d}
    config = client.update_configuration(params)
    # Set publication rate
    rate = rospy.Rate(50) # 100hz
    pub_codo_cmd.publish(0.0)

    reset_simulation()
    # Create a subscriber for codo

    unpause_simulation()


    # Main loop
    # We'll publish commands at the rate defined before


    print(time.time()-T_abs)
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

            #elevar al cuadrado
            data2=[thisData**2 for thisData in data]
            #se remueve el primer dato
            data2.remove(data2[0])

            #calcular la diferencia de tiempo por el error al cuadrado
            ponderados=[thisTiempo*thisData for thisTiempo,thisData in zip(np.diff(tiempo),data2)]
            print(sum(ponderados))

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
        run_once(100.0,2.0,3.0,'hombro')
    except rospy.ROSInterruptException:
        pass
