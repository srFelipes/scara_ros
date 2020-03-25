#!/usr/bin/python


#Este script tiene las dos funciones que se comunican al encoder.

# leer(spi,cs) entrega la posicion absoluta del encoder asociado al gpio (cs) cs={cs_hombro,cs_codo,cs_z}

#abs2offset(spi,cs,dato) llama a leer(...) para calcular el offset en base a la lectura del encoder en cs. Dato es una tupla [angulo,offset].
#La variable asociada al offset es odrv0.axis0.encoder.config.offset.

#Procedimiento:     (de preferencia tener el motor con lectura de encoder en 180° o ponerlo en una posicion donde pueda moverse a ambos lados y asociar este valor a 180 por spi
#encender odrive 
#leer posicion angular del encoder Ej: In [1]: dato[0]=leer(spi,cs_z)         
#Realizar secuencia de calibración enconder Ej: In [2]: odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION 
#Copiar offset obtenido a dato Ej: In [1]: dato[1]=odrv0.axis0.encoder.config.offset
#Guardar dato de forma permanente 

#En una nueva inicialización
 #cargar dato (Quizas guardar en spitest (?))
# Actualizar offset según abs2offset  Ej: In[1]: odrv0.axis0.encoder.config.offset=abs2offset(spi,cs_z,dato)
#setear encoder a listo Ej: In[1]: odrv0.axis0.encoder.is_ready=true
#done!

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
    #if abs(current-dato[0])>180:
    #    current = current +360 
        
    return dato[1]+(current-dato[0])/360.0*4000
        
