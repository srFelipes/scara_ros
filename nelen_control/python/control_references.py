#!/usr/bin/env python
import math
import numpy as np
from scipy import signal

def step_reference(t, min_angle, max_angle, period, duty=0.5):
    # compute amplitude
    A = max_angle - min_angle
    # compute_vertical_offset
    off = (max_angle + min_angle)/2
    # compute frequency
    f = 2*math.pi/period
    # return step reference at time t
    return (A*signal.square(f*t, duty) + off)