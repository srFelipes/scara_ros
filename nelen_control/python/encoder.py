#!/usr/bin/python

import spidev
import time
import RPi.GPIO as GPIO



# Funcion que lee el puerto spi y retorna angulo

class encoder:
    def __init__(self,cs,spi):
        GPIO.setup(cs  , GPIO.OUT)
        GPIO.output(cs , GPIO.HIGH)
        self.spi = spi

        self.spi.open(0, 0)
        self.spi.mode=0b01
        self.spi.max_speed_hz = 1000000
        self.cs=cs




    def leer(self):

        GPIO.output(self.cs, GPIO.LOW)
        msb = 0xFF
        lsb = 0xFF
        self.spi.xfer([msb, lsb])
        GPIO.output(self.cs, GPIO.HIGH)
        GPIO.output(self.cs, GPIO.LOW)
        raw=self.spi.readbytes(2)
        final=(((raw[0]<<8)+raw[1]) & 0b0011111111111111)/16383.0*360.0
        GPIO.output(self.cs, GPIO.HIGH)

        return final

    def abs2offset(self,dato):
        current=self.leer()
        if abs(current-dato[0])>180.0:
            current = current +360.0

        return dato[1]+(current-dato[0])/360.0*4000.0
        
