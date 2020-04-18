#!/usr/bin/env python

import math
import rospy
import numpy as np
from scipy import signal
from tqdm import tqdm

# import Float64 msg
from std_msgs.msg import Float64
# import control msg
from control_msgs.msg import JointControllerState

# Class particles for PSO
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

def costFunc(x):
  # simulate with Gazebo
  total_time = 2 # time to reach set_point [seg]
  try:
      error = main_loop(x, total_time) # run gazebo simulation and return overshot value
  except rospy.ROSInterruptException:
      pass
  return error

# Step reference uses angles in radians
def step_reference(t, min_angle, max_angle, period, duty=0.5):
    # compute amplitude
    A = max_angle - min_angle
    # compute_vertical_offset
    off = (max_angle + min_angle) / 2
    # compute frequency
    f = 2 * math.pi / period
    # return step reference at time t
    return (A * signal.square(f * t, duty) + off)


def codo_callback(msg):
    print("Codo state")
    print("  angle: %f " % np.rad2deg(msg.process_value))
    print("  velocity: %f " % np.rad2deg(msg.process_value_dot))
    print("  Controller config:")
    print("    set_point: %f " % msg.set_point)
    print("    command: %f " % msg.command)
    print("    error: %f " % msg.error)
    print("    time_step: %f " % msg.time_step)
    print("    kp: %f " % msg.p)
    print("    ki: %f " % msg.i)
    print("    kd: %f " % msg.d)
    print("    i_clamp: %f " % msg.i_clamp)
    print("    antiwindup: %f " % msg.antiwindup)

    # here we can do more stuff
    # blabla


def main_loop(parameter, total_time):
    # RESTAR SIMULATION. ALWAYS THE SAME INITIAL CONDITION

    # Init node
    rospy.init_node('nelen_step_response_example')

    # Create publisher for codo
    pub_codo_cmd = rospy.Publisher('/nelen/codo_control/command', Float64, queue_size=10)

    # Create a subscriber for codo
    sub_codo_state = rospy.Subscriber('/nelen/codo_control/state', JointControllerState, codo_callback)

    # Set publication rate
    rate = rospy.Rate(100)  # 100hz

    # Set PID gains. Should fix this, seems not working.. dynamic_reconfigure?
    rospy.set_param('/nelen/codo_control/pid/p', parameter[0])  # set P gain
    rospy.set_param('/nelen/codo_control/pid/i', parameter[1])  # set I gain
    rospy.set_param('/nelen/codo_control/pid/d', parameter[2])  # set D gain

    # Main loop
    # We'll publish commands at the rate defined before
    #while not rospy.is_shutdown():
    t_ini = rospy.get_time()
    t = rospy.get_time()
    # run simulation for fixed amount of time
    while t - t_ini <= total_time:
        t = rospy.get_time()  # get current time

        # Define a step response function
        angle_min = np.deg2rad(-45)
        angle_max = np.deg2rad(45)
        T = total_time
        # compute command
        codo_cmd = step_reference(t, angle_min, angle_max, T)
        # Maybe we can include other function similar to step_reference.
        # The idea is to start always with the same IC and reach a fixed set_point. Also we can include
        # a trajectory function for finding best PID gain values related to this task

        rospy.logwarn(angle_min)
        rospy.logwarn(angle_max)
        rospy.logwarn(codo_cmd)
        # publish command
        pub_codo_cmd.publish(codo_cmd)

        # micro pause to reach the desired rate
        rate.sleep()

    error = # This should be related to the error through simulation. something like  np.sum(sub_codo_state.error) or overshot
    # Check if we have received messages
    # If we had, the callback will be called and the state of the codo will be printed
    rospy.spin()
    return error

iter_num = 3 # number of iterations
particle_num = 2 # particle size
bounds_params = [[1,50],[1,50],[1,50]]
pso=PSO(costFunc=costFunc,pos_best_g=[],err_best_g=1,bounds=bounds_params,N=particle_num,iter=iter_num,c1=1,c2=2)
