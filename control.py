import numpy as np

def angle_pos_to_control(angle, pos):
    angle_car = np.arctan2(pos[1], pos[0])
    if angle_car < 0:
        angle_car += 2*np.pi
    angle_car += np.pi/2
    while angle_car > 2*np.pi:
        angle_car -= 2*np.pi
    return angle_car
