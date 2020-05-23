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
GPIO.setmode(GPIO.BCM)
#MCP
import MCP230XX as MCP
#encoder
import encoder


class NELEN():
    def __init__():
    ## Variables del robot
    #Aqui se define el SCARA

    #variables de estado
    
    self.is_home= False
    self.ls={
        "hombro": [False,False],
        "codo":   [False,False],
        "z":      [False,False],
        "a":      [False,False]
    }
    self.xpos=0.
    self.ypos=0.
    self.zpos=0.
    self.apos=0.

    self.hombro_enc_val
    self.codo_enc_val
    self.z_enc_val
    
    

    #variables OD
    self.id_rc=335E31703536
    print("buscando Odrive RC")
    self.odrv_rc=odrive.find_any(serial_number=id_rc)

    #direcciones I2C
    self.ad_mcp=0x20
    self.mcp=MCP('MCP23017', ad_mcp, '16bit')
    

    #ENCODERSS

    #abrir el puerto spi
    self.spi = spidev.SpiDev()
    self.spi.open(0, 0)
    self.spi.mode=0b01
    self.spi.max_speed_hz = 1000000
    #crear elementos de la clase encoder
    self.enc_hombro=encoder.encoder(self.spi,13)
    self.enc_codo=encoder.encoder(self.spi,16)
    self.enc_z=encoder.encoder(self.spi,19)
    self.enc_a=encoder.encoder(self.spi,20)

    #funciones callback para ls
    def mcp_callback_h(self,channel):
        #este es el callback del pin de interrupt del mcp, actualmente solo se actualiza un valor del diccionario ls, se está probando solo con un ls
        
        print("ls touched")
        binario_ls=self.mcp.single_access_read(0x12)
        self.ls["z"][0]=binario_ls>>0 & 1
        #incluir aqui el resto de los ls conectados al mcp


    #Definiciones GPIO

    self.interrupt_ls=24
    
    GPIO.setup(interrupt_ls,GPIO.IN)
    GPIO.add_event_detect(BUTTON_GPIO, GPIO.FALLING, callback=mcp_callback, bouncetime=100)

    #homing  n}}}n                                                   


