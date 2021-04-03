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
wait = 2
th_1, th_2 = ang_cur
th1, th2, z_new = None, None, None

def mov_xy(th1, th2):
    odrvh.axis0.controller.move_incremental(th1, False)
    odrvc.axis1.controller.move_incremental(th2, False)

def mov_z(znew):
    odrvc.axis0.controller.move_incremental(znew, False)

def info(th1, th2, z_new):
    print(f'Absolute theta_1 = {th1}')
    print(f'Absolute theta_2 = {th2}')
    print(f'Absolute z = {z_new}')    

def help_info():
    print('Commands:')
    print('s --> subir 100 mm EE')
    print('0 --> EE en condicion inicial (668.15, 0, 0)')
    print('s --> subir 100 mm EE')
    print('b --> EE a basurero de botellas')
    print('t --> EE a basurero de tetrapack')
    print('l --> EE a basurero de latas')
    print('b --> EE a basurero de botellas')
    print('1 --> EE a lata')
    print('2 --> EE a botelle chica')
    print('3 --> EE a tetrapack')
    print('4 --> EE a botella')
    print('Ejemplo. desde algun basurero: 1 -> (ventosa) -> s -> l')

while True:
    order = input('say what?')
    print(f'order: {order}, type: {type(order)}')
    # b: botellas trash, t: tetrapack trash, l: latas
    # lata: 1, botella chica: 2, caja: 3, botella: 4
    # subir = s
    if order == 's':
        z_new = -100
        znew = (z_new-z_cur)/16
        mov_z(znew)
        th1 = (th_1-ang_cur[0])*7/(2*3.1415)
        th2 = (th_2-ang_cur[1])*7/(2*3.1415)
    elif order == '0':
        x_new = 668.15
        y_new = 0
        z_new = 0
        th_1, th_2 = ik(x = x_new, y = y_new, L1 = L1, L2 = L2, verbose = True)
        th1 = (th_1-ang_cur[0])*7/(2*3.1415)
        th2 = (th_2-ang_cur[1])*7/(2*3.1415)
        znew = (z_new-z_cur)/16
        mov_xy(th1, th2)
        time.sleep(wait)
        mov_z(znew)
    elif order == 'b':
        x_new = 50
        y_new = 600
        th_1, th_2 = ik(x = x_new, y = y_new, L1 = L1, L2 = L2, verbose = True)
        th1 = (th_1-ang_cur[0])*7/(2*3.1415)
        th2 = (th_2-ang_cur[1])*7/(2*3.1415)
        mov_xy(th1, th2)
    elif order == 't':
        x_new = 150
        y_new = 350
        th_1, th_2 = ik(x = x_new, y = y_new, L1 = L1, L2 = L2, verbose = True)
        th1 = (th_1-ang_cur[0])*7/(2*3.1415)
        th2 = (th_2-ang_cur[1])*7/(2*3.1415)
        mov_xy(th1, th2)             
    elif order == 'l':
        x_new = 1
        y_new = 350
        th_1, th_2 = ik(x = x_new, y = y_new, L1 = L1, L2 = L2, verbose = True)
        th1 = (th_1-ang_cur[0])*7/(2*3.1415)
        th2 = (th_2-ang_cur[1])*7/(2*3.1415)
        mov_xy(th1, th2)
    elif order == '1':
        x_new = 668.15
        y_new = 0
        z_new = 15
        znew = (z_new-z_cur)/16
        th_1, th_2 = ik(x = x_new, y = y_new, L1 = L1, L2 = L2, verbose = True)
        th1 = (th_1-ang_cur[0])*7/(2*3.1415)
        th2 = (th_2-ang_cur[1])*7/(2*3.1415)
        mov_xy(th1, th2)
        time.sleep(wait)
        mov_z(znew)
    elif order == '2':
        x_new = 500
        y_new = 60
        z_new = 15
        znew = (z_new-z_cur)/16
        th_1, th_2 = ik(x = x_new, y = y_new, L1 = L1, L2 = L2, verbose = True)
        th1 = (th_1-ang_cur[0])*7/(2*3.1415)
        th2 = (th_2-ang_cur[1])*7/(2*3.1415)
        mov_xy(th1, th2)
        time.sleep(wait)
        mov_z(znew)
    elif order == '3':
        x_new = 600
        y_new = 200
        z_new = 17
        znew = (z_new-z_cur)/16
        th_1, th_2 = ik(x = x_new, y = y_new, L1 = L1, L2 = L2, verbose = True)
        th1 = (th_1-ang_cur[0])*7/(2*3.1415)
        th2 = (th_2-ang_cur[1])*7/(2*3.1415)
        mov_xy(th1, th2)
        time.sleep(wait)
        mov_z(znew)
    elif order == '4':
        x_new = 460
        y_new = 250
        z_new = -18
        znew = (z_new-z_cur)/16
        th_1, th_2 = ik(x = x_new, y = y_new, L1 = L1, L2 = L2, verbose = True)
        th1 = (th_1-ang_cur[0])*7/(2*3.1415)
        th2 = (th_2-ang_cur[1])*7/(2*3.1415)
        mov_xy(th1, th2)
        time.sleep(wait)
        mov_z(znew)
    elif order == 'help':
        help_info()
    else:
        print('Not included yet!')
    info(th1, th2, z_new)
    ang_cur = th_1, th_2
    z_cur = z_new
