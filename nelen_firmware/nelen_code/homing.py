#!/home/rosconcoco/nelen/bin/python
import odrive
from odrive.enums import * 
from odrive.utils import *

odrvc = odrive.find_any(serial_number="207135A1524B") # codo and z
odrvh = odrive.find_any(serial_number="206E3591524B") # hombro

# Settings
#odrvc.axis0.controller.config.pos_gain = 13 #saved
#odrvc.axis0.controller.config.vel_limit = 200 #saved
#odrvc.axis0.trap_traj.config.vel_limit = 200 #saved
#odrvc.axis1.controller.config.vel_limit = 4.35/1.5 #saved
#odrvc.axis1.trap_traj.config.vel_limit = 4.35/1.5 #saved
#odrvc.axis1.controller.config.inertia = 0.17/4 #saved
#odrvh.axis0.controller.config.vel_limit = 2.5/1.5 #saved
#odrvh.axis0.trap_traj.config.vel_limit = 2.5/1.5 #saved
#odrvh.axis0.controller.config.inertia = 0.84/8 #saved 

#odrvc.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ #saved
#odrvc.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ #saved
#odrvh.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ #saved

#odrvh.axis0.min_endstop.config.debounce_ms = 50 #saved

# Offset update
#odrvc.axis0.min_endstop.config.offset = .5 #saved
#odrvc.axis1.min_endstop.config.offset = .2 #saved
#odrvh.axis0.min_endstop.config.offset = .2 #saved

# Then.. 
import time

print('Dumping errors codo-muneca')
dump_errors(odrvc, True)
print('Dumping errors hombro')
dump_errors(odrvh, True)

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

