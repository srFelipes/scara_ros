# !/usr/bin/python3

from __future__ import print_function
import odrive
from odrive.enums import *
# from odrive.utils import start_liveplotter
import time
import math
# Raspy
import signal
import sys
import RPi.GPIO as GPIO

## Inicializar odrives, ver estados de axis y encoders

# Find an ODrive that is connected on the serial port /dev/ttyUSB0
#my_drive = odrive.find_any("serial:/dev/ttyUSB0")

def def_mot(ID, nombre, home, vel, LS_steps, LS, touch = 0):
    '''ID = ID odrv
    axis_i = axis asociado al motor que se quiere inicializar
    nombre = nombre de la articulacion
    home = 1, realizar secuancia de
    LS = numero LS GPIO asociado a la articulacion que se quiere homiar'''
    # callback LS
    def LS_callback(channel):
        touch = 1
    # inicializar LS
    # https://roboticsbackend.com/raspberry-pi-gpio-interrupts-tutorial/
    GPIO.setup(LS, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # VERIFICAR
    GPIO.add_event_detect(LS, GPIO.RISING,
                              callback=LS_callback, bouncetime=100)  # VERIFICAR bouncetime
    print("Buscando Odrive asociado a: " + str(nombre))
    odrv_i = odrive.find_any(ID)
    print(str(nombre) + " axis errors:")
    if nombre == hombro or nombre == z:
        mot_i = odrv_i.axis0
    if nombre == codo:
        mot_i = odrv_i.axis1
    print(str(nombre) + " axis encoder errors:")
    encdr_i = mot_i.encoder
    print("motor is calibrated = " + str(mot_i.motor.is_calibrated))
    if home == 1:
        #Aqui puede incluirse el offset para no volver a calibrar
        if mot_i.motor.is_calibrated == 0 and mot_i.encoder.is_read == 0:
            mot_i.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            print("waiting for encoder offset calibration to end..")
            while mot_i.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
            print("State: " + str(mot_i.current_state))
            print("encoder is ready = " + str(mot_i.encoder.is_ready))  # Esta listo el encoder?
            print("entering close loop control mode..")
        mot_i.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL  # entrar en modo closed loop control
        print("State: " + str(mot_i.current_state))
        mot_i.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL  # controlar por velocidad para llegar a LS
        while True:
            if not touch:
                mot_h.controller.vel_setpoint = vel
            else:
                break
        mot_i.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL  # volver a controlar por posicion (devolverse del LS
        mot_i.controller.pos_setpoint = (mot_i.encoder.pos_estimate + LS_steps)
        print("Position setpoint is " + str(mot_i.controller.pos_setpoint))
        print(str(nombre) + "is in home")
    return [odrv_i, mot_i, encdr_i]

def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

GPIO.setmode(GPIO.BCM)

signal.signal(signal.SIGINT, signal_handler)

# Hombro
ID_h = ""
#vel_h = -100
#pasos_LS_h = 100
#LS_h = # LS para homing. VERIFICAR
#[odrv_h, mot_h, encdr_h] = def_mot(ID_h, hombro, 1, vel_h, pasos_LS_h, LS_h)  # para calibrar

# Codo
ID_cz = ""
#vel_c = VERIFICAR
#pasos_LS_c = VERIFICAR
#LS_c =  # LS para homing. VERIFICAR
#[odrv_cz, mot_c, encdr_c] = def_mot(ID_cz, codo, 1, vel, pasos_LS_c, LS_c) # para calibrar

# Z
#vel_z = VERIFICAR
#pasos_LS_z = VERIFICAR
# LS_z =  # LS para homing. VERIFICAR
#[mot_z, encdr_z] = def_mot(ID_cz, z, 1, vel, pasos_LS_z, LS_z)[1:] # para calibrar

#LS_break =   # definir esta variable como el 'or' entre todos los LS.
# Cortar en caso de emergencia, ignorando esto en el homing
