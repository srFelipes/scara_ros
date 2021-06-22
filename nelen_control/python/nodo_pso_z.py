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
data_z     =[]
tiempo_z   =[]
accion_z   =[]

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

def escalon(t):
    t_onset = 0.5
    if t<t_onset:
        return 0.
    else:
        return 1.


#Generar trayectoria de posicionamiento
back=planning.trayectoria(0.20,A_max=29.6,V_max=2.,J_max=592,S_max=4000.)
#generar trayectoria rango completo

keep=np.linspace(0.,1.,1000)

global totalCurve
totalCurve   = back[0]+[back[0][-1] for ke in keep]
# total_codo   = [-3.14/2*0.94*escalon(t) for t in np.linspace(0,2,1000)]+[-3.14/2*0.94 for ke in keep]+[-3.14/2*0.94+3.14*0.94*escalon(t) for t in np.linspace(0,2,1000)]+[3.14/2*0.94 for ke in keep]
# total_hombro =  [-15.0/18.0*3.14*0.94*escalon(t) for t in np.linspace(0,2,1000)]+[-15.0/18.0*3.14*0.94 for ke in keep]+[-15.0/18.0*3.14*0.94+15.0/9.0*3.14*0.94*escalon(t) for t in np.linspace(0,2,1000)]+[15.0/18.0*3.14*0.94 for ke in keep]


def tray(t):
    global totalCurve
    ind=t*500.0
    if ind+1>=len(totalCurve):
        return totalCurve[-1]
    elif ind==int(ind):
        return totalCurve[int(ind)]
    else:
        return totalCurve[int(ind)]*(1-ind+int(ind))+totalCurve[int(ind)+1]*(ind-int(ind))



def z_callback(msg):
    #print('guardardato')

    #se guarda el error
    data_z.append(msg.error)

    #se guarda el tiempo
    tiempo_z.append(msg.header.stamp.to_sec())
    accion_z.append(msg.command)

#initialize ros stuff
rospy.init_node('run_simulation_once')
client_z     = dynamic_reconfigure.client.Client('/nelen/prism_control/pid')
pub_z_cmd   = rospy.Publisher('/nelen/prism_control/command', Float64, queue_size=10)
sub_z_state = rospy.Subscriber('/nelen/prism_control/state', JointControllerState, z_callback)


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



def run_once(pid, fun):
    global totalCurve
    t_it=len(totalCurve)/500.0+0.25  #the whole realization+0.25s

    # Setear nuevas ganancias
    params ={'p':pid[0],'i':pid[1],'d':pid[2]}
    config = client_z.update_configuration(params)
        # frecuencia de publicacion
    rate = rospy.Rate(100) # 100hz
    pub_z_cmd.publish(0.0)

    #resetear resultados
    data_z[:]=[]
    tiempo_z[:]=[]
    accion_z[:]=[]
    reset_simulation()
    unpause_simulation()
    time.sleep(0.01)
    t_init=rospy.get_time()
    while True:
        t = rospy.get_time() # tiempo actual
        # evaluar en funciones de trayectoria
        pub_z_cmd.publish(fun(t-t_init))
        if t>=t_it+t_init:

            #pausar simulacion y cambiar el set point  a valor inicial
            pause_simulation()
            pub_z_cmd.publish(0.0)
            #retornar las variables donde se guardan los resultados
            return ([tiempo_z,data_z])

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



  resultados=run_once(x[0:3],tray)



  operado   = errorCuadratico(resultados[0],resultados[1])
  # op_action_codo   = errorCuadratico(resultados[2],resultados[5])
  # op_action_hombro = errorCuadratico(resultados[0],resultados[4])


  if operado<0.0:
      print(resultados[0])
      print(resultados[2])
      pause_simulation()
      time.sleep(0.5)
      reset_simulation()
      time.sleep(0.5)
      resultados=run_once(x[0:3],tray)
      operado   = errorCuadratico(resultados[0],resultados[1])

      if operado<0.0:
          print(resultados[0])
          print(resultados[2])
          return 10000
      else:
          # print(100.*(operado_codo+operado_hombro)+(op_action_codo+op_action_hombro)/1000.)
          return (operado)
  else:
      # print x,
      # print(operado)
      # print((operado_codo+operado_hombro)+(op_action_codo+op_action_hombro)/1000.)
      return (operado)





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


        # #
        # # # #programa final
        iter_num      = 20 # number of iterations
        particle_num  = 25# particle size
        bounds_params = [[0.0,70000.0],[0.0,5000.],[0.0,500.0]]

        for thisS in [50.,80.,160.,1600.,4000.,8000.]:
            global totalCurve

            back=planning.trayectoria(-0.20,A_max=29.6,V_max=2.,J_max=592,S_max=thisS)
            #generar trayectoria rango completo
            keep=np.linspace(0.,1.,500)

            totalCurve   = back[0]+[back[0][-1] for ke in keep]

            pso=PSO(costFunc=costFunc,pos_best_g=[],err_best_g=1,bounds=bounds_params,N=particle_num,iter=iter_num,c1=1,c2=2)
            print(pso.pos_best_g)
            #
            resultados=run_once(pso.pos_best_g,tray)
            # resultados=run_once([715.68699424, 408.18883589,0.],[442.2078436,156.67895682,20.16516559],codo_tray,hombro_tray)

            sp=[tray(t) for t in resultados[0]]
            print(len(sp)-len(resultados[0]))

            pv=[dis_sp-er for er,dis_sp in zip(resultados[1],sp)]

            plt.subplot(3,1,1)

            plt.plot(resultados[0],sp)
            plt.plot(resultados[0],pv)
            plt.subplot(3,1,2)
            plt.plot(resultados[0],resultados[1])
            plt.subplot(3,1,3)
            plt.plot(resultados[0],accion_z)

            plt.savefig('z_'+str(int(thisS))+'.png')
            plt.clf()








        # pso=PSO(costFunc=costFunc,pos_best_g=[],err_best_g=1,bounds=bounds_params,N=particle_num,iter=iter_num,c1=1,c2=2)
        # print(pso.pos_best_g)







    except rospy.ROSInterruptException:
        pass
