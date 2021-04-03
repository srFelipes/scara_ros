import numpy as np

def ik(x, y, L1, L2, orientation = 'izq', verbose = False):
    '''Calculate inverse kinematic of the arm:
    input:  
    x = x target position
    y = y target position
    L1 = humero lenght
    L2 = radio-cubito lenght
    output: 
    theta1 (hombro degree) theta2 (codo degree)'''
    in_arccos = min(max((x**2 + y**2 - L1**2 - L2**2)/(2*L1*L2),-1) ,1)
    if orientation == 'izq':
        theta_2 = np.arccos(in_arccos)
        theta_1 = np.arctan(y/x) - np.arctan(L2*np.sin(theta_2)/(L1 + L2*np.cos(theta_2))) 
    elif orientation == 'der':
        theta_2 = -np.arccos(in_arccos)
        theta_1 = np.arctan(y/x) + np.arctan(L2*np.sin(theta_2)/(L1 + L2*np.cos(theta_2))) 
    else:
        print('Orientation mode error. It should be equal to: str(der) or str(izq)')
    if verbose == True:
        print(f'Targets: x = {x}, y = {y}')
        print(f'Theta_1 = {theta_1}, Theta_2 = {theta_2}')
    return theta_1, theta_2
