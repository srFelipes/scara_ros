#!/usr/bin/env python
import time
# Se importan paquetes importantes
import math
import rospy
import numpy as np
from tqdm import tqdm
import copy
import matplotlib.pyplot as plt

import s_curve_trayectory as planning
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
data_codo     =[]
tiempo_codo   =[]
data_hombro   =[]
tiempo_hombro =[]
accion_codo   =[]
accion_hombro =[]
# tiempo_ref    =[]
# ref_codo      =[]
# ref_hombro    =[]

def cubic_ref(not_t):
    #referencia cubica
    t_init=0.5
    t=not_t+t_init
    tf=0.666
    a0=-3/2.0*0.9420
    a_end=3.0/2.0*0.9420

    if t<=tf:
        a2=3.0*(a_end-a0)/(tf**2)
        a3=-2.0*(a_end-a0)/(tf**3)
        return a0+a2*(t**2)+a3*(t**3)
    elif t<0.0: return a0
    else: return a_end

#Generar trayectoria de posicionamiento
back=planning.two_joints(-3.14/2*0.94,-15.0/18.0*3.14*0.94)
#generar trayectoria rango completo
full=planning.two_joints(3.14*0.94,30.0/18.0*3.14*0.94)
keep=np.linspace(0.,1.,1000)

total_codo   = back[4]+[back[4][-1] for ke in keep]+[fu+back[4][-1] for fu in full[4]]+[full[4][-1]+back[4][-1] for ke in keep]
total_hombro = back[0]+[back[0][-1] for ke in keep]+[fu+back[0][-1] for fu in full[0]]+[full[0][-1]+back[0][-1] for ke in keep]


def codo_tray(t):
    ind=t*500.0
    if t<0.:
        return total_codo[0]
    elif ind+1>=len(total_codo):
        return total_codo[-1]
    elif ind==int(ind):
        return total_codo[int(ind)]
    else:
        return total_codo[int(ind)]*(1-ind+int(ind))+total_codo[int(ind)+1]*(ind-int(ind))

def hombro_tray(t):
    ind=t*500.0
    if t<0.:
        return total_hombro[0]
    elif ind+1>=len(total_hombro):
        return total_hombro[-1]
    elif ind==int(ind):
        return total_hombro[int(ind)]
    else:
        return total_hombro[int(ind)]*(1-ind+int(ind))+total_hombro[int(ind)+1]*(ind-int(ind))

def codo_callback(msg):
    #print('guardardato')

    #se guarda el error
    data_codo.append(msg.process_value)

    #se guarda el tiempo
    tiempo_codo.append(msg.header.stamp.to_sec())
    accion_codo.append(msg.command)

def hombro_callback(msg):
    #print('guardardato')

    #se guarda el error
    data_hombro.append(msg.process_value)

    #se guarda el tiempo
    tiempo_hombro.append(msg.header.stamp.to_sec())
    accion_hombro.append(msg.command)

#initialize ros stuff
rospy.init_node('run_simulation_once')
client_codo      = dynamic_reconfigure.client.Client('/nelen/codo_cont/pid')
client_hombro    = dynamic_reconfigure.client.Client('/nelen/hombro_control/pid')
pub_codo_cmd     = rospy.Publisher('/nelen/codo_cont/command', Float64, queue_size=10)
sub_codo_state   = rospy.Subscriber('/nelen/codo_cont/state', JointControllerState, codo_callback)
pub_hombro_cmd   = rospy.Publisher('/nelen/hombro_control/command', Float64, queue_size=10)
sub_hombro_state = rospy.Subscriber('/nelen/hombro_control/state', JointControllerState, hombro_callback)


class Particle:
    def __init__(self, x, c1, c2):
        self.dim = len(x)
        self.pos_i = x
        self.vel_i = np.random.uniform(-1, 1, self.dim)
        self.pos_best_i = []
        self.err_best_i = 100000000
        self.err_i = 100000000
        self.c1 = c1
        self.c2 = c2

    def evaluate(self, costFunc):
        self.err_i = costFunc(self.pos_i) # evaluate parameter's performance

        # update pos_best_i and err_best_i
        if self.err_i < self.err_best_i:
            self.pos_best_i = self.pos_i
            self.err_best_i = self.err_i

    def update_vel(self, pos_best_g, w):
        r1 = np.random.random(self.dim)
        r2 = np.random.random(self.dim)
        vel_cognitive = self.c1 * r1 * (np.array(self.pos_best_i) - np.array(self.pos_i))
        vel_social = self.c2 * r2 * (np.array(pos_best_g) - np.array(self.pos_i))
        self.vel_i = w * self.vel_i + vel_cognitive + vel_social

    def update_pos(self, bounds):
        self.pos_i += self.vel_i

        for i in range(self.dim):
            if self.pos_i[i] > bounds[i][1]:
                self.pos_i[i] = bounds[i][1]
            if self.pos_i[i] < bounds[i][0]:
                self.pos_i[i] = bounds[i][0]
class PSO():
    def __init__(self, costFunc, bounds, N, iter, c1, c2, pos_best_g=[], err_best_g=1):
        self.err_log = np.zeros((N, iter))
        xdim = len(bounds)

        self.pos_best_g = pos_best_g

        self.err_best_g = err_best_g

        swarm = []
        for i in range(N):
            xx = []
            for d in range(xdim):
                xx.append(np.random.uniform(bounds[d][0], bounds[d][1]))
            swarm.append(Particle(xx, c1, c2))

        print("Birds are exploring...")
        i = 0

        pbar = tqdm(total=iter * N)
        while i < iter:
            w = np.exp(-i / iter)

            for j in range(N):
                swarm[j].evaluate(costFunc)
                self.err_log[j, i] = swarm[j].err_i

                if swarm[j].err_i < self.err_best_g:
                    self.pos_best_g = copy.deepcopy(swarm[j].pos_i)
                    self.err_best_g = copy.deepcopy(swarm[j].err_i)
                pbar.update(1)
            print("Best position so far : ", self.pos_best_g)
            print("Best error so far : ", self.err_best_g)

            for j in range(N):
                swarm[j].update_vel(self.pos_best_g, w)
                swarm[j].update_pos(bounds)

            i += 1
        pbar.close()

        print("Iteration :", iter)
        print("Optimized parameters :", self.pos_best_g)
        print("Minimum error :", self.err_best_g)

#esta funcion se opera cada vez que el nodo recibe un mensaje

def sigmoide(t):
    t_tot = 2
    t_onset = 0.5
    agres = 25.0
    expon = 1/(1 + np.exp(-(t - t_onset) * agres))

    return expon


def run_once(hombro_pid,codo_pid, fun_codo,fun_hombro):

    t_it=len(total_codo)/500.0+0.25  #the whole realization+0.25s

    # Set the new gains
    params_hombro ={'p':hombro_pid[0],'i':hombro_pid[1],'d':hombro_pid[2]}
    config_hombro = client_hombro.update_configuration(params_hombro)
    params_codo   ={'p':codo_pid[0],'i':codo_pid[1],'d':codo_pid[2]}
    config_codo   = client_codo.update_configuration(params_codo)
    # Set publication rate
    rate = rospy.Rate(1000) # 100hz
    pub_codo_cmd.publish(0.0)

    #resetear resultados
    data_codo[:]=[]
    tiempo_codo[:]=[]
    data_hombro[:]=[]
    tiempo_hombro[:]=[]
    accion_codo[:]=[]
    accion_hombro[:]=[]
    reset_simulation()
    unpause_simulation()
    time.sleep(0.01)
    t_init=rospy.get_time()
    while True:
        t = rospy.get_time() # get current time
        # Ver que publicar o retornar
        # tiempo_ref.append(t-t_init)


        pub_codo_cmd.publish(fun_codo(t-t_init))
        # ref_codo.append(fun_codo(t-t_init))
        pub_hombro_cmd.publish(fun_hombro(t-t_init))
        # ref_hombro.append(fun_hombro(t-t_init))
        if t>=t_it+t_init:

            #pausar simulacion y cambiar el set point
            pause_simulation()
            pub_codo_cmd.publish(0.0)
            pub_hombro_cmd.publish(0.0)

            return ([tiempo_hombro,data_hombro,tiempo_codo,data_codo])

        # elif t>=t_escalon+t_init:
        #     pub_codo_cmd.publish(1.0)
        #
        # else:
        #     pub_codo_cmd.publish(0.0)


        # micro pause to reach the desired rate
        rate.sleep()



def ponderar(resultado):
    lamda=20.0
    time_back=len(back[0])/500.0
    time_keep=len(keep)/500.0
    time_full=len(full[0])/500.0
    el_k=0
    # print 'largos =' ,
    # print len(resultado),
    # print len(resultado[0]),
    # print len(resultado[1]),
    # print len(resultado[2]),
    # print len(resultado[3])
    this_hombro=[0. for l in resultado[1]]
    this_codo=[0. for l in resultado[3]]
    for t in resultado[0][0:-1]:
        if t>time_back and t<time_keep+time_back:
            this_hombro[el_k]=resultado[1][el_k]*lamda
        elif t>time_keep+time_back+time_full:
            this_hombro[el_k]=resultado[1][el_k]*lamda
        else:
            this_hombro[el_k]=resultado[1][el_k]

        # print 'el k hombro fue ',
        # print el_k,
        # print ' largo hombro = ',
        # print len(this_hombro),
        # print ' largo tiempo = ',
        # print len(resultado[1])
        el_k=el_k+1


    el_k=0
    for t in resultado[2][0:-1]:
        if t>time_back and t<time_keep+time_back:
            this_codo[el_k]=resultado[3][el_k]*lamda
        elif t>time_keep+time_back+time_full:
            this_codo[el_k]=resultado[3][el_k]*lamda
        else:
            this_codo[el_k]=resultado[3][el_k]
        # print 'el k codo fue ',
        # print el_k,
        # print ' largo codo = ',
        # print len(this_hombro),
        # print ' largo tiempo = ',
        # print len(resultado[3])
        el_k=el_k+1


    return [resultado[0],this_hombro,resultado[2],this_codo]

def costFunc(x):
  # simulate with Gazebo with parameters x=[p,i,d]
  # let's add a delay of d seconds
  d=0.0
  #Run simulation once
  sin_ponder   =run_once(x[0:3],x[3:6],codo_tray,hombro_tray)
  #calculate plant set point
  refer_hombro =[hombro_tray(t-d) for t in sin_ponder[0]]
  # print(refer_hombro)
  # print([hombro_tray(t) for t in sin_ponder[0]])
  # plt.plot(refer_hombro)
  # plt.plot([hombro_tray(t) for t in sin_ponder[0]])
  # plt.plot(sin_ponder[1])
  # plt.show()
  refer_codo   =[codo_tray(t-d) for t in sin_ponder[2]]
  #calculate the error with a delay in the reference
  sin_ponder[1]=[sp-pv for sp,pv in zip(refer_hombro,sin_ponder[1])]
  sin_ponder[3]=[sp-pv for sp,pv in zip(refer_hombro,sin_ponder[3])]


  resultados=ponderar(sin_ponder)

  operado_codo     = errorCuadratico(resultados[2],resultados[3])
  operado_hombro   = errorCuadratico(resultados[0],resultados[1])
  # op_action_codo   = errorCuadratico(resultados[2],resultados[5])
  # op_action_hombro = errorCuadratico(resultados[0],resultados[4])


  if operado_codo<0.0 or operado_hombro<0.0:
      print(resultados[0])
      print(resultados[2])
      pause_simulation()
      time.sleep(0.5)
      reset_simulation()
      time.sleep(0.5)
      resultados=run_once(x[0:3],x[3:6],codo_tray,hombro_tray)
      operado_codo= errorCuadratico(resultados[2],resultados[3])
      operado_hombro= errorCuadratico(resultados[0],resultados[1])
      if operado_codo<0.0 or operado_hombro<0.0:
          print(resultados[0])
          print(resultados[2])
          return 10000
      else:
          # print(100.*(operado_codo+operado_hombro)+(op_action_codo+op_action_hombro)/1000.)
          return (operado_codo+operado_hombro)
  else:
      print x,
      print(operado_codo+operado_hombro)
      # print((operado_codo+operado_hombro)+(op_action_codo+op_action_hombro)/1000.)
      return (operado_codo+operado_hombro)





def errorCuadratico(tiempo,data):
  # alpha=1.2
  # data2=ponderar(data,alpha)
  data2=[thisData**2 for thisData in data]
     #se remueve el primer dato
  data2.remove(data2[0])

     #calcular la diferencia de tiempo por el error al cuadrado
  ponderados=[thisTiempo*thisData for thisTiempo,thisData in zip(np.diff(tiempo),data2)]
  return sum(ponderados)/len(ponderados)

if __name__ == '__main__':
    try:

        # run_once([350.0,5.0,15.0],[350.0,5.0,15.0],codo_tray,hombro_tray)



        # #programa final
        iter_num      = 5 # number of iterations
        particle_num  = 50 # particle size
        bounds_params = [[0.0,1000.0],[0.0,500.0],[0.0,500.0],[0.0,1000.],[0.,500.],[0.,500.0]]
        pso=PSO(costFunc=costFunc,pos_best_g=[],err_best_g=100,bounds=bounds_params,N=particle_num,iter=iter_num,c1=1,c2=2)
        print(pso.pos_best_g)

        resultados=run_once(pso.pos_best_g[0:3],pso.pos_best_g[3:6],codo_tray,hombro_tray)
        # costFunc([552.0537093,167.40174294,372.27289947,0.,500.,449.16874596])
        # resultados=run_once([100., .18883589,10.],[100.2078436,.67895682,20.16516559],codo_tray,hombro_tray)

        hombro_sp=[hombro_tray(t) for t in resultados[0]]
        hombro_pv=resultados[1]
        codo_sp=[codo_tray(t) for t in resultados[2]]
        codo_pv=resultados[3]
        plt.subplot(3,2,1)
        plt.plot(resultados[0],hombro_sp)
        plt.plot(resultados[0],hombro_pv)
        plt.subplot(3,2,3)
        plt.plot(resultados[0],resultados[1])
        plt.subplot(3,2,2)
        plt.plot(resultados[2],codo_sp)
        plt.plot(resultados[2],codo_pv)
        plt.subplot(3,2,4)
        plt.plot(resultados[2],resultados[3])
        plt.subplot(3,2,5)
        plt.plot(resultados[0],accion_hombro)
        plt.subplot(3,2,6)
        plt.plot(resultados[2],accion_codo)
        plt.show()
        # print(costFunc([100.,0.,0.,100.,0.,0.]))
        # plt.plot(tiempo_hombro,data_hombro)
        # plt.plot(tiempo_ref,ref_hombro)
        # plt.show()





        # pso=PSO(costFunc=costFunc,pos_best_g=[],err_best_g=1,bounds=bounds_params,N=particle_num,iter=iter_num,c1=1,c2=2)
        # print(pso.pos_best_g)







    except rospy.ROSInterruptException:
        pass
