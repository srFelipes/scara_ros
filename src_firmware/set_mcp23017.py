#!/usr/bin/python


#Codgio para setear la configuracion del MCP23017
#Basado en el codigo de Owain Martin
from MCP230XX import MCP230XX

i2cAddress=0x20     #direccion del dispositivo, correspondiente a todos los pines de direccion a tierra
MCP = MCP230XX('MCP23017', i2cAddress, '16bit') # se inicializa el objeto MCP

#setear los pines 0-7 (A) como entrada

MCP.set_mode(0,'input')
MCP.set_mode(1,'input')
MCP.set_mode(2,'input')
MCP.set_mode(3,'input')
MCP.set_mode(4,'input')
MCP.set_mode(5,'input')
MCP.set_mode(6,'input')
MCP.set_mode(7,'input')

#setear los interrupt
MCP.add_interrupt(0)
MCP.add_interrupt(1)
MCP.add_interrupt(2)
MCP.add_interrupt(3)
MCP.add_interrupt(4)
MCP.add_interrupt(5)
MCP.add_interrupt(6)
MCP.add_interrupt(7)


#setear interrupt en modo de comparacion respecto a defval

MCP.single_access_write(0x08,0b11111111)

#setear el valor de defval
#cambiar por 1 los pines que estan conectados a limit switches Normally Closed (NC)
MCP.single_access_write(0x06,0b00000000)

#leer pines
print MCP.input(0)
print MCP.input(1)
print MCP.input(2)
print MCP.input(3)
print MCP.input(4)
print MCP.input(5)
print MCP.input(6)
print MCP.input(7)

#leer todos los pines juntos

print bin(MCP.single_access_read(0x12))
