#!/usr/bin/env python

"""
script para leer temperatura cada 5 segundos
"""
from time import sleep
from mlx90614 import MLX90614

sensor1=MLX90614(0x2f)
sensor2=MLX90614(0x5b)
sensor3=MLX90614(0x5c)

while True:
    print '2F [amb= ',
    print(sensor1.get_amb_temp()),
    print ' | obj = ',
    print(sensor1.get_obj_temp()),
    print '] ',
    print '5B [amb= ',
    print(sensor2.get_amb_temp()),
    print ' | obj = ',
    print(sensor2.get_obj_temp()),
    print '] ',
    print '5C [amb= ',
    print(sensor1.get_amb_temp()),
    print ' | obj = ',
    print(sensor1.get_obj_temp()),
    print ']'
    sleep(2)
    
    
    
