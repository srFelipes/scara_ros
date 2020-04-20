#!/usr/bin/env python
import numpy as np
import math
import matplotlib.pyplot as plt

def cubic_ref(t,tf,a0,a_end):
    #referencia cubica

    #tf=1.5
    # a0=-3/2.0
    # a_end=3.0/2.0

    if t<0.0:
        return [a0, 0.0]
    elif t<=tf:
        a2=3.0*(a_end-a0)/(tf**2)
        a3=-2.0*(a_end-a0)/(tf**3)
        return [a0+a2*(t**2)+a3*(t**3), 2*a2*t+3*a3*(t**2)]
    else:
        return [a_end, 0.0]

#Esta funcion se utiliza para calcular la trayectoria optima
def vel_total(q1,q2,q1d,q2d):
    r1=0.33
    r2=0.33
    xd=-r1*np.sin(q1)*q1d-r2*np.sin(q2+q1)*(q1d+q2d)
    yd=r1*np.cos(q1)*q1d+r2*np.cos(q2+q1)*(q1d+q2d)
    return np.sqrt(xd**2+yd**2)


lim1=3.1416/2
lim2=15.0*3.1416/18.0
safety=0.9420
duration=0.666

x = np.linspace(-0.1, duration+0.1, 1000)
teta1=[cubic_ref(t,duration,-lim1*safety,lim1*safety) for t in x]
teta2=[cubic_ref(t,duration,-lim2*safety,lim2*safety) for t in x]

vel=[]
ind=0
for t in x:
    vel.append(vel_total(teta1[ind][0],teta2[ind][0],teta1[ind][1],teta2[ind][1]))
    ind=ind+1

print(max(vel))
#plt.plot(x, vel)

vel1=[]
vel2=[]
ind=0
for v in x:
    vel1.append(teta1[ind][1])
    vel2.append(teta2[ind][1])
    ind=ind+1


plt.plot(x,vel1)

plt.plot(x,vel2)
plt.xlabel("time [s]")
plt.ylabel("total speed [m/s]")

plt.show()
