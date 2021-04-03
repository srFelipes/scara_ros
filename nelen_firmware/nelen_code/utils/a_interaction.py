from smbus2 import SMBus, i2c_msg

def movea(address, step):
    ''' Move A motor step steps, using i2c protocol'''
    with SMBus(1) as bus:
        msg = i2c_msg.write(address, [0, step])
        bus.i2c_rdwr(msg)

def reada(address):
    '''Read A position with encoder'''
    with SMBus(1) as bus:
        msg = i2c_msg.read(address, 2)
        pos = bus.i2c_rdwr(msg)
        print(f'Encoder position: {pos}')
