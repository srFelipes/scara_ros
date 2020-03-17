#!/usr/bin/python

import spidev
import time
import RPi.GPIO as GPIO

spi = spidev.SpiDev()
spi.open(0, 0)
spi.mode=0b01
spi.max_speed_hz = 1000

# Use "GPIO" pin numbering
GPIO.setmode(GPIO.BCM)


#numero de pin de los CS asociados a los distintos encoders
cs_hombro = 16
cs_codo   = 20
cs_z      = 19
cs_a      = 13

# setear como salidas

GPIO.setup(cs_hombro , GPIO.OUT)
GPIO.setup(cs_codo   , GPIO.OUT)
GPIO.setup(cs_z      , GPIO.OUT)
GPIO.setup(cs_a      , GPIO.OUT)

# apagar salidas
GPIO.output(cs_hombro , GPIO.HIGH)
GPIO.output(cs_codo   , GPIO.HIGH)
GPIO.output(cs_z      , GPIO.HIGH)
GPIO.output(cs_a      , GPIO.HIGH) 

# Funcion que lee el puerto spi y retorna angulo
def leer(spi,cs):

    GPIO.output(cs, GPIO.LOW)
    msb = 0xFF
    lsb = 0xFF
    spi.xfer([msb, lsb])
    GPIO.output(cs, GPIO.HIGH)
    GPIO.output(cs, GPIO.LOW)
    raw=spi.readbytes(2)
    final=(((raw[0]<<8)+raw[1]) & 0b0011111111111111)/16383.0*360.0
    GPIO.output(cs, GPIO.HIGH)
    
    return final

def abs2offset(spi,cs,dato):
    current=leer(spi,cs)
    if abs(current-dato[0])>180:
        current = current +360 
        
    return dato[1]+(current-dato[0])/360.0*4000
        
