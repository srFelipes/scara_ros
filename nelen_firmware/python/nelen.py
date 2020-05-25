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

    #medidas
        self.largo_humero=0.3
        self.largo_radio_cubito=0.3
        self.paso_z=16.0/1000.0

        #limites en +- excepto z

        self.recorrido_hombro  =90.0
        self.recorrido_codo    =150.0
        self.recorrido_z       =0.35



        #variables de estado

        self.Ts = 1./8000.

        self.is_homed     = False
        self.iscalibrated = False
        self.ls={
            "hombro": [False,False],
            "codo":   [False,False],
            "z":      [False,False],
            "a":      [False,False]
            }

            #posicion
            self.xpos = 0.
            self.ypos = 0.
            self.zpos = 0.
            self.apos = 0.

            self.hombro_enc_val = 0.
            self.codo_enc_val   = 0.
            self.z_enc_val      = 0.
            self.a_enc_val      = 0.

            self.hombro_vueltas = 0.
            self.codo_vueltas   = 0.
            self.z_vueltas      = 0.
            self.a_vueltas      = 0.

            self.hombro_cal_val = 0.
            self.codo_cal_val   = 0.
            self.z_cal_val      = 0.
            self.a_cal_val      = 0.

            #velocidad
            self.hombro_enc_vel = 0.
            self.codo_enc_vel   = 0.
            self.z_enc_vel      = 0.

            self.hombro_vel = 0.
            self.codo_vel   = 0.
            self.z_vel      = 0.




            #variables OD
            self.id_rc=335E31703536
            print("buscando Odrive RC")
            self.odrv_rc=odrive.find_any(serial_number=id_rc)
            # self.id_hombro=335E31703536
            # print("buscando Odrive hombro")
            # self.odrv_hombro=odrive.find_any(serial_number=id_hombro)


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
            if self.ls["z"][0]:
                #falta setear velocidad a 0

        #incluir aqui el resto de los ls conectados al mcp



        #Definiciones GPIO

        self.interrupt_ls=24

        GPIO.setup(interrupt_ls,GPIO.IN)
        GPIO.add_event_detect(BUTTON_GPIO, GPIO.FALLING, callback=mcp_callback, bouncetime=100)

        #Calibrar
    def calibrar(self,caso='all'):
        #<axis>.config.startup_encoder_offset_calibration
        #^
        #se puede cambiar esta variable a true en el odriva
        #asi se corre la rutina al prender el od
        if caso=='all':
            self.odrv_hombro.axis0.requested_state=AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            print("waiting for hombro encoder offset calibration to end..")
            while self.odrv_hombro.axis0.current_state != AXIS_STATE_IDLE:
                time.sleep(0.5)
            print("motor hombro calibrado")

            print("waiting for codo encoder offset calibration to end..")
            self.odrv_rc.axis0.requested_state=AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            while self.odrv_rc.axis0.current_state != AXIS_STATE_IDLE:
                time.sleep(0.5)
            print("motor codo calibrado")

            print("waiting for z encoder offset calibration to end..")
            self.odrv_rc.axis1.requested_state=AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            while self.odrv_rc.axis1.current_state != AXIS_STATE_IDLE:
                time.sleep(0.5)
            print("motor z calibrado")
        if caso=='z':
            self.odrv_rc.axis0.requested_state=AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    def homming(self,caso):
        #secuencia de homming
        if caso=='z':
            self.odrv_rc.axis0.controller.config.control_mode=CTRL_MODE_VELOCITY_CONTROL
            self.odrv_rc.axis0.requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL

    def one_sample(self):
        self.hombro_enc_val = self.enc_hombro.leer()
        self.codo_enc_val   = self.enc_codo.leer()
        self.z_enc_val      = self.enc_z.leer()
        self.a_enc_val      = self.enc_a.leer()
