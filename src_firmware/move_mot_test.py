#!/usr/bin/python3

from __future__ import print_function

import odrive
from odrive.enums import *
#from odrive.utils import start_liveplotter
import time
import math

print("Buscando Odrive for the Duuudes!")
odrv_wis = odrive.find_any()
print(str(odrv_wis.vbus_voltage))
print("axis errors:")
print(str(odrv_wis.axis0.error))
print("axis motor errors:")
print(str(odrv_wis.axis0.motor.error))
print("axis encder errors:")
print(str(odrv_wis.axis0.encoder.error))
#print(str(dump_errors(odrv_wis, True)))
#print(str(odrv0.axis0))
#print(str(odrv0.axis0.motor))
#print(str(odrv0.axis0.encoder))

print("Estado: " + str(odrv_wis.axis0.current_state))
# Esta calibrado el motor?
print("motor is calibrated = " + str(odrv_wis.axis0.motor.is_calibrated))

# encontrar offset
print("starting calibration")
odrv_wis.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
print("waiting for encoder offset calibration to end..")
while odrv_wis.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
print("Estado: " + str(odrv_wis.axis0.current_state))
print("encoder is ready = " + str(odrv_wis.axis0.encoder.is_ready)) # Esta listo el encoder?

#entrar a modo control
odrv_wis.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
#print("waiting for closed loop control strategy..")
#while odrv_wis.axis0.current_state != AXIS_STATE_ENCODER_OFFSET_CALIBRATION:
#    time.sleep(0.1)
print("Estado: " + str(odrv_wis.axis0.current_state))

pasos = 700
odrv_wis.axis0.controller.pos_setpoint = (odrv_wis.axis0.encoder.pos_estimate + pasos)
print("Position setpoint is " + str(odrv_wis.axis0.controller.pos_setpoint))

# llevar motor a LS y setear meta-vuelta = 0
# Leer sobre kinematica inversa


# plot
#def liveplot():
#    start_liveplotter(lambda: [odrv0.axis0.encoder.pos_estimate])

#liveplot()
#start_liveplotter(lambda: [odrv0.axis0.encoder.pos_estimate])



