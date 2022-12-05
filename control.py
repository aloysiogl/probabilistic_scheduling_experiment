import numpy as np

def angle_pos_to_control(angle, pos):
    angle_car = np.arctan2(pos[1], pos[0])
    if angle_car < 0:
        angle_car += 2*np.pi
    angle_car += np.pi/2
    while angle_car > 2*np.pi:
        angle_car -= 2*np.pi
    return angle_car

   
def get_control(pid, env, car_index): 
    pos = env.sim.agents[car_index].state[0:2]
    steer = -pid(np.linalg.norm(pos))
    return steer