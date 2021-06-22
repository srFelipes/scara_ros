#!/usr/bin/env python
# Se importan paquetes importantes
import math
import rospy
import numpy as np
from tqdm import tqdm
import copy
import matplotlib.pyplot as plt
import time

import s_curve_trayectory as planning
# import Float64 msg
from std_msgs.msg import Float64
# import control msg
from sensor_msgs.msg import JointState

#necesario para llamar a los servicions de pausa despausa y reinicializacion
from std_srvs.srv import Empty

#paquete para cambiar los parametros del PID
import dynamic_reconfigure.client
import sys

#variables universales para guardar los datos de la realziacion
data_codo     =[]
tiempo        =[]
data_hombro   =[]
action_hombro =[]
action_codo   =[]
codo_at       =0.
hombro_at     =0.

published= False
t_init_published= False
t_init=-1.

def tray(t,full):
    ind=t*500.0
    # print(ind-len(full[0]))
    if ind+1>=len(full[0]):
        return [full[0][-1],full[4][-1]]
    elif ind==int(ind):
        return [full[0][int(ind)],full[4][int(ind)]]
    else:
        return [full[0][int(ind)]*(1-ind+int(ind))+full[0][int(ind)+1]*(ind-int(ind)),full[4][int(ind)]*(1-ind+int(ind))+full[4][int(ind)+1]*(ind-int(ind))]

def callback(msg):
    # print('guardardato')
    global published
    global t_init
    global t_init_published
    global hombro_at
    global codo_at
    # print(t_init_published)

    if t_init_published:
        # print(t_init_published)
        #se guarda el error
        data_hombro.append(msg.position[1])
        data_codo.append(msg.position[0])
        action_hombro.append(msg.effort[1])
        action_codo.append(msg.effort[0])

        #se guarda el tiempo
        tiempo.append(msg.header.stamp.to_sec()-t_init)
    else:
        # print 'guardo_hombro'
        hombro_at=(msg.position[1])
        codo_at=(msg.position[0])

    published=True


#initialize ros stuff


def main_loop(hombro,codo,S):
    # Set publication rate
    global t_init
    global t_init_published
    rospy.init_node('run_tray')
    pub_codo_cmd     = rospy.Publisher('/nelen/codo_cont/command', Float64, queue_size=10)
    subscriber       = rospy.Subscriber('/nelen/joint_states', JointState, callback)
    pub_hombro_cmd   = rospy.Publisher('/nelen/hombro_control/command', Float64, queue_size=10)


    rate = rospy.Rate(500) # 500hz
    while not published:
        #print('waiting for the message')

        pass

    c0=codo_at
    h0=hombro_at
    # print('hombro')
    # print(float(hombro))

    full=planning.two_joints(float(hombro)-h0,float(codo)-c0,float(S))
    full[0]=[f+h0 for f in full[0]]
    full[4]=[f+c0 for f in full[4]]

    #get the initial time

    t_end=len(full[0])/500.+0.25
    # print('un')
    time.sleep(.5)
    # print('seg')
    t_init = rospy.get_time()
    t_init_published= True
    while not rospy.is_shutdown():
        t = rospy.get_time() # get current time
        #Calculate the next position
        command=tray(t-t_init,full)
        coman1=(command[1])
        coman0=(command[0])
        # publish command

        pub_codo_cmd.publish(coman1)
        pub_hombro_cmd.publish(coman0)

        if t-t_init>t_end:
            print('theend')
            subscriber.unregister()
            return full
        # micro pause to reach the desired rate
        rate.sleep()



if __name__ == '__main__':
    try:
        if len(sys.argv) < 4:
            print("usage: run_tray.py hombro codo S")
            print("hombro and codo are the final set points")
        else:
            ful=main_loop(sys.argv[1], sys.argv[2], sys.argv[3])
            hombro_sp=[tray(t,ful)[0] for t in tiempo]
            hombro_error=[sp-pv for pv,sp in zip(data_hombro,hombro_sp)]
            codo_sp=[tray(t,ful)[1] for t in tiempo]
            codo_error=[sp-pv for pv,sp in zip(data_codo,codo_sp)]
            # plt.subplot(3,2,1)
            # plt.title('hombro')
            # plt.plot(tiempo,hombro_sp)
            # plt.plot(tiempo,data_hombro)
            # plt.subplot(3,2,3)
            # plt.plot(tiempo,hombro_error)
            # plt.subplot(3,2,2)
            # plt.title('codo')
            # plt.plot(tiempo,codo_sp)
            # plt.plot(tiempo,data_codo)
            # plt.subplot(3,2,4)
            # plt.plot(tiempo,codo_error)
            # plt.subplot(3,2,5)
            # plt.plot(tiempo,action_hombro)
            # plt.xlabel('tiempo [s]')
            # plt.subplot(3,2,6)
            # plt.plot(tiempo,action_codo)
            # plt.xlabel('tiempo [s]')
            # plt.show()


    except rospy.ROSInterruptException:
        pass
