#!/usr/bin/env python

import math
import rospy
import numpy as np
from scipy import signal

# import Float64 msg
from std_msgs.msg import Float64
# import control msg
from control_msgs.msg import JointControllerState

# Step reference uses angles in radians
def step_reference(t, min_angle, max_angle, period, duty=0.5):
    # compute amplitude
    A = max_angle - min_angle
    # compute_vertical_offset
    off = (max_angle + min_angle)/2
    # compute frequency
    f = 2*math.pi/period
    # return step reference at time t
    return (A*signal.square(f*t, duty) + off)

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

def main_loop():
    rospy.init_node('nelen_step_response_example')
    
    # Create publisher for codo
    pub_codo_cmd = rospy.Publisher('/nelen/codo_cont/command', Float64, queue_size=10)

    # Create a subscriber for codo
    sub_codo_state = rospy.Subscriber('/nelen/codo_cont/state', JointControllerState, codo_callback)

    # Set publication rate
    rate = rospy.Rate(100) # 100hz

    # Main loop
    # We'll publish commands at the rate defined before
    while not rospy.is_shutdown():
        t = rospy.get_time() # get current time
        
        # Define a step response function
        angle_min = np.deg2rad(-45)
        angle_max = np.deg2rad(45)
        T = 10
        # compute command
        codo_cmd = step_reference(t, angle_min, angle_max, T)
        rospy.logwarn(angle_min)
        rospy.logwarn(angle_max)
        rospy.logwarn(codo_cmd)
        # publish command
        pub_codo_cmd.publish(codo_cmd)

        # micro pause to reach the desired rate
        rate.sleep()

    # Check if we have received messages
    # If we had, the callback will be called and the state of the codo will be printed
    rospy.spin()
    
if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass