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

# import bayesian optimization library
from bayes_opt import BayesianOptimization

# import control references
import control_references as cr

from functools import partial


# This class setups all the settings to control gazebo in a structured way
class GazeboControlExperiment:
    def __init__(self, joint, trialTime, setpointFunction, rate=50):
        self.joint = joint
        self.trialTime = trialTime
        self.setpointFunction = setpointFunction

        self.rate = rospy.Rate(rate) # 50hz

        # prepare structures to save data
        self.control_error=[]
        self.control_time=[]

        # gazebo control services
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.pause_simulation = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_simulation = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

        # prepare all the publishers
        self.codo_client = dynamic_reconfigure.client.Client('/nelen/codo_control/pid')
        self.codo_pub_cmd = rospy.Publisher('/nelen/codo_control/command', Float64, queue_size=10)
        self.codo_sub_state = rospy.Subscriber('/nelen/codo_control/state', JointControllerState, self.listener_callback)

        self.hombro_client = dynamic_reconfigure.client.Client('/nelen/hombro_control/pid')
        self.hombro_pub_cmd = rospy.Publisher('/nelen/hombro_control/command', Float64, queue_size=10)
        self.hombro_sub_state = rospy.Subscriber('/nelen/hombro_control/state', JointControllerState, self.listener_callback)

        self.z_client = dynamic_reconfigure.client.Client('/nelen/prism_control/pid')
        self.z_pub_cmd = rospy.Publisher('/nelen/prism_control/command', Float64, queue_size=10)
        self.z_sub_state = rospy.Subscriber('/nelen/prism_control/state', JointControllerState, self.listener_callback)

    def listener_callback(self, msg):
        #se guarda el error
        self.control_error.append(msg.error)
        #se guarda el tiempo
        self.control_time.append(msg.header.stamp.to_sec())

    def publish_zero_command(self):
        self.codo_pub_cmd.publish(0.0)
        self.hombro_pub_cmd.publish(0.0)
        self.z_pub_cmd.publish(0.0)

    def publish_command(self, cmd):
        if self.joint=='codo':
            self.codo_pub_cmd.publish(cmd)
            self.hombro_pub_cmd.publish(0.0)
            self.z_pub_cmd.publish(0.0)

        elif self.joint=='hombro':
            self.codo_pub_cmd.publish(0.0)
            self.hombro_pub_cmd.publish(cmd)
            self.z_pub_cmd.publish(0.0)

        elif self.joint=='z':
            self.codo_pub_cmd.publish(0.0)
            self.hombro_pub_cmd.publish(0.0)
            self.z_pub_cmd.publish(cmd)
        else:
            print('not a joint')
            exit(-1)

    def update_pid_parameters(self, p, i, d):
        params={'p': p,'i': i,'d': d}

        if self.joint=='codo':
            self.config = self.codo_client.update_configuration(params)
        elif self.joint=='hombro':
            self.config = self.hombro_client.update_configuration(params)
        elif self.joint=='z':
            self.config = self.z_client.update_configuration(params)
        else:
            print('not a joint')
            exit(-1)
        

    def run_trial(self, p, i, d):
        #rospy.loginfo("Starting trial with parameters P: %f, I: %f, D: %f" % (p,i,d))
        # Update parameters in parameter server
        self.update_pid_parameters(p, i, d)

        # reset gazebo
        self.reset_simulation()
        self.unpause_simulation()
        time.sleep(1)

        # set all the joints at 0.0
        self.publish_zero_command()

        # reset the structures to save data
        self.control_error=[]
        self.control_time=[]

        # prepare iterations
        t_init=rospy.get_time()
        t = rospy.get_time()
        while True:
            if rospy.is_shutdown():
                rospy.logfatal("Killing the node...")
                exit(-1)
            
            t = rospy.get_time() # get current time
            #rospy.loginfo("Current time %f" % (t))
            if (t > t_init + self.trialTime):
                break
            
            # prepare command using setpoint function
            cmd = self.setpointFunction(t-t_init)
            # publish command
            self.publish_command(cmd)
            
            # sleep to adjust to rate
            self.rate.sleep()

        # measure the finish time    
        t_end = rospy.get_time()

        # pause simulation
        self.pause_simulation()
        
        # Print time
        #print("Total time of the trial: %f s" % (t_end-t_init))

        # when the trial is over, compute the error cost (SUM error^2*dt)
        # remove first number
        self.control_error.remove(self.control_error[0])
        # compute the error
        #error_cost = sum([dt*e*e for dt, e in zip(np.diff(self.control_time), self.control_error)])
        
        error_cost = 0
        for dt, e in zip(np.diff(self.control_time), self.control_error):
            if(dt < 0):
                print("dt: %f, error: %f, term: %f" % (dt, e, dt*e*e))
            error_cost = error_cost + dt*e*e

        # return cost
        # we put a minus sign because BO tries to maximize a function
        # the error cost is  a sum of squares, which has a minimum at zero
        # hence, minimizing the error is maximizing the negative cost
        return -(error_cost) 


# define setpoint function, must depend only on t
# partial allows to create a new function from another one setting some of the inputs
# in this case we set everything but t
setpoint = partial(cr.step_reference, min_angle=np.deg2rad(45), max_angle= np.deg2rad(45), period=10, duty=0.5)

# main
if __name__ == '__main__':
    try:
        rospy.init_node('optimize_pid_bo')

        # read parameters
        joint = rospy.get_param('~joint', 'codo')
        trial_time = rospy.get_param('~trial_time', 10)
        trials = int(rospy.get_param('~trials', 20))

        print("Parameters")
        print(" Selected joint: %s" % joint)
        print(" Time of each trial: %f s" % trial_time)
        print(" Number of trials: %f" % trials)

        # Creamos experimento
        experiment = GazeboControlExperiment(joint, trial_time, setpoint)

        # Creamos optimizador bayesiano
        pbounds = {'p': (0.1, 10), 'i': (0.1, 10), 'd': (0.1, 10)}

        optimizer = BayesianOptimization(
            f=experiment.run_trial,
            pbounds=pbounds,
            random_state=1
        )

        # Optimize
        initial_points=0
        optimizer.maximize(init_points=initial_points, n_iter=trials)

        # Show best solution
        print("Best solution:\n cost: %f \n p: %f \n i: %f \n d: %f" % (optimizer.max["target"], optimizer.max["params"]["p"], optimizer.max["params"]["i"], optimizer.max["params"]["d"]))

    except rospy.ROSInterruptException:
        pass
