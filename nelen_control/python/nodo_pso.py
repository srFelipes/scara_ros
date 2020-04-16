#!/usr/bin/env python
import time
# Se importan paquetes importantes
import math
import rospy
import numpy as np
from tqdm import tqdm
import copy

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

#
class Particle:
    def __init__(self, x, c1, c2):
        self.dim = len(x)
        self.pos_i = x
        self.vel_i = np.random.uniform(-1, 1, self.dim)
        self.pos_best_i = []
        self.err_best_i = 1
        self.err_i = 1
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
            condition = True
            pub_codo_cmd.publish(0.0)

            #elevar al cuadrado
            data2=[thisData**2 for thisData in data]
            #se remueve el primer dato
            data2.remove(data2[0])

            #calcular la diferencia de tiempo por el error al cuadrado
            ponderados=[thisTiempo*thisData for thisTiempo,thisData in zip(np.diff(tiempo),data2)]
            return (sum(ponderados))

        elif t>=t_escalon+t_init:
            pub_codo_cmd.publish(1.0)

        else:
            pub_codo_cmd.publish(0.0)


        # micro pause to reach the desired rate
        rate.sleep()

    # Check if we have received messages
    # If we had, the callback will be called and the state of the codo will be printed
    #rospy.spin()

def costFunc(x):
  # simulate with Gazebo with parameters x=[p,i,d]
  jointAentrenar='hombro'
  return run_once(x[0],x[1],x[2],jointAentrenar)

if __name__ == '__main__':
    try:
        iter_num = 3 # number of iterations
        particle_num = 2 # particle size
        bounds_params = [[1,50],[1,50],[1,50]]
        pso=PSO(costFunc=costFunc,pos_best_g=[],err_best_g=1,bounds=bounds_params,N=particle_num,iter=iter_num,c1=1,c2=2)
        print(pso.pos_best_g)
    except rospy.ROSInterruptException:
        pass
