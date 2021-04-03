#!/home/rosconcoco/nelen/bin/python
import odrive
import time
#import tinyik
from utils.inverse_kinematic import ik
from utils.a_interaction import movea

odrvc = odrive.find_any(serial_number="207135A1524B") # codo and z
odrvh = odrive.find_any(serial_number="206E3591524B") # hombro

L1 = 330.15 # Humero [mm]
L2 = 338.0 # Radio-cubito [mm] 

#arm = tinyik.Actuator(['z', [L1, 0.0, 0.0], 'z', [L2, 0.0, 0.0]])

x0 = 668.15
y0 = 0
z0 = 0

# reset encoder. Proceed with caution!
#odrvh.axis0.encoder.set_linear_count(0)
#odrvc.axis0.encoder.set_linear_count(0)
#odrvc.axis1.encoder.set_linear_count(0)

#def x_to_turn(x, turn_per_deg)

print('Initial conditions:')
#print(f'Angles: {arm.angles}')
#print(f'XYZ: {arm.ee}')
ang_cur = ik(x = x0, y = y0, L1 = L1, L2 = L2)
print(f'Angles: {ang_cur}')
print(f'XYZ: {x0, y0, z0}')
z_cur = 0
a_cur = 0
absolute = float(input('absolute mode? (yes, type 1. no, type 0) '))
mult_sleep = 0
address = 0x69 # Arduino ID 
mova = False

while True:
# include if LS
    #xyz_cur = arm.ee
    #ang_cur = arm.angle
    #print(f'Current angles: {arm.angles}')
    #print(f'Current XYZ: {arm.ee}')
    

    # mov = input('relative == 0  and absolute == 1')
    x_new = float(input('Gimme an x, Dude! [all in mm!] '))
    y_new = float(input('And for y? '))
    z_new = float(input('What about z? '))
    if mova:
        a_new = float(input('Finally, a (degree)? '))
    #arm.ee = [x_new, y_new, z_new]
    if absolute:
        #th1, th2 = arm.angles
        th_1, th_2 = ik(x = x_new, y = y_new, L1 = L1, L2 = L2, verbose = True)
        th1 = (th_1-ang_cur[0])*7/(2*3.1415)
        th2 = (th_2-ang_cur[1])*7/(2*3.1415)
        znew = (z_new-z_cur)/16
        if mova:
            anew = int((a_new-a_cur)/360*200)
        print(f'Absolute theta_1 = {th1}')
        print(f'Absolute theta_2 = {th2}')
        print(f'Absolute z = {z_new}')
        if mova:
            print(f'Absolute a = {a_new}')
    else:
        print('not implemented yet, Dude!')
        break
    ang_cur = th_1, th_2
    z_cur = z_new
    if mova:
        a_cur = a_new 
    # hombro
    odrvh.axis0.controller.move_incremental(th1, False)
    time.sleep(1.5*mult_sleep)
    # codo
    odrvc.axis1.controller.move_incremental(th2, False)
    time.sleep(1.5*mult_sleep)
    # z
    odrvc.axis0.controller.move_incremental(znew, False)
    time.sleep(1.5*mult_sleep)
    # a
    if mova:
        movea(address, anew)
        time.sleep(1.5*mult_sleep)
