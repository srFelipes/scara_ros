#!/home/rosconcoco/nelen/bin/python
from utils.a_interaction import movea
import time

mult_sleep = 0
a_cur = 0

address = 0x69
while True:
    a_new = int(input('move A to (degree)? '))
    anew = int((a_new-a_cur)/360*200)
    print(f'Absolute a = {a_new}')
    a_cur = a_new 
    # a
    movea(address, anew)
    time.sleep(1.5*mult_sleep)
