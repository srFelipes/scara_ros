#!/home/rosconcoco/nelen/bin/python
# To connect directly to a specific odrive:
import odrive
from odrive.enums import * 
from odrive.utils import *

odrvc = odrive.find_any(serial_number="207135A1524B") # codo and z
odrvh = odrive.find_any(serial_number="206E3591524B") # hombro

# Then.. 
import time

print('Dumping errors codo-muneca')
dump_errors(odrvc, True)
print('Dumping errors hombro')
dump_errors(odrvh, True)

'''
# TODO: Generalize calibration and close_loop_control access

## Codo-muneca
# axis0
print('Working on z')
current_state_0 = odrvc.axis0.current_state
while (current_state_0 == 1):
    odrvc.axis0.requested_state = 7
    time.sleep(12)
    odrvc.axis0.requested_state = 11
    time.sleep(15)
    current_state_0 = odrvc.axis0.current_state
    if current_state_0 == 1:
        print('Current axis successfully homed')
        odrvc.axis0.requested_state = 8
        current_state_0 = odrvc.axis0.current_state
        time.sleep(1)
        if current_state_0 == 8: 
            print('Current axis successfully enters control mod3')
    else:
        print(f'Control mode access failed. Current axis state = {current_state_0}')
        dump_errors(odrvc, True)
        time.sleep(1)

# axis1
print('Working on codo')
current_state_1 = odrvc.axis1.current_state
while (current_state_1 == 1):
    odrvc.axis1.requested_state = 7
    time.sleep(12)
    odrvc.axis1.requested_state = 11
    time.sleep(15)
    current_state_1 = odrvc.axis1.current_state
    if odrvc.axis1.current_state == 1:
        print('Current axis successfully homed')
        odrvc.axis1.requested_state = 8
        time.sleep(1)
        current_state_1 = odrvc.axis1.current_state
        if current_state_1 == 8:  
            print('Current axis successfully enters control mod3')
    else:
        print(f'Control mode access failed. Current axis state = {current_state_1}')
        dump_errors(odrvc, True)
        time.sleep(1)

## Hombro
print('Working on Hombro')
# axis 0
current_state_h = odrvh.axis0.current_state
while (current_state_h == 1):
    odrvh.axis0.requested_state = 7
    time.sleep(12)
    odrvh.axis0.requested_state = 11
    time.sleep(15)
    current_state_h = odrvh.axis0.current_state
    if current_state_h == 1:
        print('Current axis successfully homed')
        odrvh.axis0.requested_state = 8
        time.sleep(1)
        current_state_h = odrvh.axis0.current_state
        if (current_state_h == 8):
            print('Current axis successfully enters control mod3')
            current_state_h = odrvh.axis0.current_state

    else:
        print(f'Homing failed. Current axis state = {current_state_h}')
        dump_errors(odrvh, True)
        time.sleep(1)

# straight
# z
offset_z_stg = -3.25 # 6.25
odrvc.axis0.controller.move_incremental(offset_z_stg, False)
time.sleep(5)
# codo
offset_c_stg = -2.6
odrvc.axis1.controller.move_incremental(offset_c_stg, False)
time.sleep(5)
# hombro
offset_h_stg = -1.75 
odrvh.axis0.controller.move_incremental(offset_h_stg, False)
time.sleep(5) 

print('Lets do it!!')
c1_axis_inter_time = .8*2 #1 axis inter time between axis movements
c2_axis_inter_time = 1.2*2
c_same_axis_inter_time = 0.3 #0.5 axis inter time between axis movements
h1_same_axis_inter_time = 0.6*2
h2_same_axis_inter_time = 0.1*3
c_codo_mov = 1
c_z_mov = 2
h_mov = 1
while (current_state_0 == 8 and current_state_1 == 8 and current_state_h == 8):
    odrvh.axis0.controller.move_incremental(h_mov, False) #  horario
    time.sleep(h1_same_axis_inter_time)  
    odrvc.axis1.controller.move_incremental(c_codo_mov, False) # horario
    time.sleep(c1_axis_inter_time)
    odrvc.axis0.controller.move_incremental(c_z_mov, False) # abajo
    time.sleep(c_same_axis_inter_time)    
    odrvc.axis0.controller.move_incremental(-c_z_mov, False) # arriba
    time.sleep(c_same_axis_inter_time)    
    odrvc.axis1.controller.move_incremental(-c_codo_mov, False) # anti-horario
    time.sleep(h2_same_axis_inter_time)
    odrvh.axis0.controller.move_incremental(-h_mov, False) # 
    time.sleep(c2_axis_inter_time)  
    odrvc.axis0.controller.move_incremental(c_z_mov, False) # abajo
    time.sleep(c_same_axis_inter_time)    
    odrvc.axis0.controller.move_incremental(-c_z_mov, False) # arriba
    time.sleep(c_same_axis_inter_time)
print('Disable motors')
print(f'Current axis state = {odrvc.axis0.current_state}')
odrvc.axis0.requested_state = 1
print(f'Current axis state = {odrvc.axis1.current_state}')
odrvc.axis1.requested_state = 1
print(f'Current axis state = {odrvh.axis0.current_state}')
odrvh.axis0.requested_state = 1
print('Dump errors codo-z')
dump_errors(odrvc)
print('Dump errors hombro')
dump_errors(odrvh)'''
