#!/usr/bin/python

while True: 
     if odrv0.system_stats.uptime>last+1000: 
         print(odrv0.axis1.encoder.pos_estimate) 
         last=odrv0.system_stats.uptime 
