# !/usr/bin/python3

from __future__ import print_function
#import odrive
#from odrive.enums import *
# from odrive.utils import start_liveplotter
import time
import math
import tinyik
# Raspy
import signal
import sys
import RPi.GPIO as GPIO

# LS Interrupt
# https://roboticsbackend.com/raspberry-pi-gpio-interrupts-tutorial/
LS_h_l =
LS_h_r =
LS_rc_l =
LS_rc_r =
LS_z_u =
LS_z_d =
LS_puntita =
LS_GPIO = [LS_h_l, LS_h_d, LS_rc_l, LS_rc_d, LS_z_u, LS_z_d, LS_puntita] # incluir todos los LS
LS_stop = False

'''Algorithm for controlling SCARA robot. 
from cartesian (x,y,z) to (x',y',z')
x axis is positive in the direction of maximum stretching
y axis is positive counterclockwise
z <= 0'''

## SCARA parameters
L1 = 330.15 # Humero [mm]
L2 = 338.0 # radio-cubito [mm]
arm = tinyik.Actuator(['z', [L1, 0., 0.], 'z', [L2, 0., 0.]])
ID_h = "" #INCLUIR
#odrv_h = odrive.find_any(ID_h)
cuentas_h_ang = 6000/360 # cuantas cuentas equivalen a un grado? [cuentas/grado]. VERIFICAR
ID_cz = "" #INCLUIR
#odrv_cz = odrive.find_any(ID_cz)
cuentas_c_ang = 6000/360 # cuantas cuentas equivalen a un grado? [cuentas/grado]. VERIFICAR
vueltas_z_mm = 2 # cuantas vueltas da por milimtro avanzado [vueltas/mm]. VERIFICAR
cuentas_z_vuelta = 6000 # cuentas cuentas realiza al girar 360 grados [cuentas/vuelta]. VERIFICAR
# Asumamos que parte en 10,0,0, producto de haber realizado rutina de homing
# ARREGAR ESTOS VALORES, TINYIK ASUME QUE EL ANGULO 0,0 EQUIVALE AL BRAZO ESTIRADO AL COMPLETO
x = 10
y = 0
z = 0
arm.ee = [x, y, z]
print('condicion inicial')
print([x, y ,z])
print('angulo inicial')
print(arm.angles)
print('--------o--------')

def mov_xy(x, y, x_old, y_old, ang_old, relative = 1):
    if relative == 1:
        print('moving x incremental ' + str(x) + '[mm] and y incremental ' + str(y) + '[mm]')
        # from x, y to angle
        x = x + x_old
        y = y + y_old
    else:
        print('moving x to ' + str(x) + '[mm] and y to ' + str(y) + '[mm]')
    arm.ee = [x, y, 0.]  # entrega IK del brazo
    h_ang, c_ang = arm.angles - ang_old
    print('angulos a mover')
    print(h_ang, c_ang)
    # from angle to counts
    h_counts = h_ang * cuentas_h_ang
    c_counts = c_ang * cuentas_c_ang
    #odrv_h.axis0.controller.pos_setpoint = (odrv_h.axis0.encoder.pos_estimate + h_counts)
    #odrv_cz.axis1.controller.pos_setpoint = (odrv_cz.axis1.encoder.pos_estimate + c_counts)
    return [x, y]

def mov_z(z, z_old, relative = 1):
    if relative == 1:
        print('moving z incremental ' + str(z) + '[mm]')
    else:
        print('moving z to ' + str(z) + '[mm]')
        z = z - z_old
    z_counts = z * (vueltas_z_mm) * (cuentas_z_vuelta)
    #odrv_cz.axis0.controller.pos_setpoint = (odrv_cz.axis0.encoder.pos_estimate + z_counts)
    return z


def mov_all(x, y, z, x_old, y_old, z_old, ang_old, relative = 1):
    #print(mov_xy(x, y, x_old, y_old, relative), mov_z(z, z_old, relative))
    return [mov_xy(x, y, x_old, y_old, ang_old,relative), mov_z(z, z_old, relative)]

# Interrupt code
def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

def LS_stop_callback(channel):
    global stop
    stop = 1

if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)

    for n in range(0, len(LS_GPIO)):
        GPIO.setup(LS_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Inicializar GPIO. VERIFICAR
        GPIO.add_event_detect(LS_GPIO[n], GPIO.RISING,
                          callback=LS_stop_callback, bouncetime=100)  # Agregar evento. VERIFICAR bouncetime


    signal.signal(signal.SIGINT, signal_handler)
    # moving position
    while True:
        if not stop:
            # update variables for next movement
            x_old = x
            y_old = y
            z_old = z
            ang_old = arm.angles
            print('angulo antiguo updated')
            print(ang_old)
            print('x,y,z antiguo updated')
            print([x_old, y_old, z_old])
            mov_type = input("movimiento relativo (1) o absoluto (0)? ")
            mov_type = int(mov_type)
            if mov_type == 1:
                print("movimiento relativo")
            else:
                print("movimiento absoluto")
            x = input("Dude, a que valor de x me muevo [mm]? ")
            x = float(x)
            print("Ir a x = ", x)
            y = input("Dude, a que valor de y me muevo[mm]? ")
            y = float(y)
            print("Ir a y = ", y)
            z = input("Dude a que valor de z me muevo[mm]? ")
            z = float(z)
            print("Ir a z = ", z)
            [[x, y], z] = mov_all(x, y, z, x_old, y_old, z_old, ang_old,mov_type)
            print('valor de x luego de aplicar mov_all')
            print(x)
            print('valor de y luego de aplicar mov_all')
            print(y)
            print('valor de z luego de aplicar mov_all')
            print(z)
            #if alarm == 1: # define alarm = 1 when one LS is active
            #    break
            print('-------o--------')

        # trayectory
        ''' # set desired (x,y,z) EE position
            x = np.arange(40, 75, 35)
            y = np.arange(10, 30, 50)
            z = np.zeros(len(x))'''


''' Tuning
Tuning the motor controller is an essential step to unlock the full potential of the ODrive. 
Tuning allows for the controller to quickly respond to disturbances or changes in the system 
(such as an external force being applied or a change in the setpoint) without becoming unstable. 
Correctly setting the three tuning parameters (called gains) ensures that ODrive can control your motors 
in the most effective way possible. The three values are:

    <axis>.controller.config.pos_gain = 20.0 [(counts/s) / counts]
    <axis>.controller.config.vel_gain = 5.0 / 10000.0 [A/(counts/s)]
    <axis>.controller.config.vel_integrator_gain = 10.0 / 10000.0 [A/((counts/s) * s)]

An upcoming feature will enable automatic tuning. Until then, here is a rough tuning procedure:

    Set vel_integrator_gain gain to 0
    Make sure you have a stable system. If it is not, decrease all gains until you have one.
    Increase vel_gain by around 30% per iteration until the motor exhibits some vibration.
    Back down vel_gain to 50% of the vibrating value.
    Increase pos_gain by around 30% per iteration until you see some overshoot.
    Back down pos_gain until you do not have overshoot anymore.
    The integrator can be set to 0.5 * bandwidth * vel_gain, where bandwidth is the overall 
    resulting tracking bandwidth of your system. Say your tuning made it track commands with a 
    settling time of 100ms (the time from when the setpoint changes to when the system arrives 
    at the new setpoint); this means the bandwidth was 1/(100ms) = 1/(0.1s) = 10hz. 
    In this case you should set the vel_integrator_gain = 0.5 * 10 * vel_gain.

The liveplotter tool can be immensely helpful in dialing in these values. 
To display a graph that plots the position setpoint vs the measured position value run the 
following in the ODrive tool:

start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])'''

