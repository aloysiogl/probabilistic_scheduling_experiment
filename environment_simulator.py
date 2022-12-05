from locale import normalize

import time

import gym
import numpy as np
import shapely.geometry as shp
from simple_pid import PID

from pyglet.gl import GL_POINTS

from trajectory_predictor.utils.SplineOptimizer import SplineOptimizer
from trajectory_predictor.controller.WallFollowerController import WallFollowerController
from control import angle_pos_to_control, get_control
from LaneChanger import LaneChanger
from scheduling.Racecar import Racecar

def add_circle_to_map(env, radius, scale):

    points = [np.array([np.cos(theta), np.sin(theta)])*radius*scale for theta in np.linspace(0, 2*np.pi, 100)]
    for p in points:
        env.batch.add(1, GL_POINTS, None, ('v3f/stream', [p[0], p[1], 0.]),
                            ('c3B/stream', [183, 193, 222]))


def get_render_callback(map_params):
    state = 0
    def render_callback(env_renderer):
            # custom extra drawing function
            nonlocal state
            state += 1
            e = env_renderer
            window_size = 1000

            # update camera to follow car
            x = e.cars[0].vertices[::2]
            y = e.cars[0].vertices[1::2]
            top, bottom, left, right = max(y), min(y), min(x), max(x)
            e.score_label.x = left
            e.score_label.y = top - window_size
            e.left = left - window_size
            e.right = right + window_size
            e.top = top + window_size
            e.bottom = bottom - window_size

            add_circle_to_map(e, 22, scale=50)



    return render_callback

def main():
    DISPLAY = True
    env = gym.make('f110_gym:f110-v0', map='./map', map_ext='.pgm', num_agents=2, timestep=0.01)
    # read map parameters
    import yaml
    with open('./map.yaml') as f:
        map_params = yaml.load(f, Loader=yaml.FullLoader)
    env.add_render_callback(get_render_callback(map_params))

    # Rad csv file with numpy
    width = 4
    inner_radius = 20

    n_lanes = 2
    lane_width = width/n_lanes
    lane_centers = np.linspace(0, lane_width*(n_lanes-1), n_lanes)+lane_width/2+inner_radius
    env.lane_centers = lane_centers
    lane_changer = LaneChanger(0, lane_centers)

    car_2_initial_angle = np.pi/6
    car_2_initial_lane = lane_centers[1]
    car_2_initial_pos = [np.cos(car_2_initial_angle)*car_2_initial_lane,
                         np.sin(car_2_initial_angle)*car_2_initial_lane, 
                         np.pi/2+car_2_initial_angle]
    obs, step_reward, done, info = env.reset(np.array([[lane_centers[0], 0, np.pi/2], car_2_initial_pos]))

    if DISPLAY:
        env.render()

    laptime = 0.0
    start = time.time()
    speed1, speed2 = 6,5
    progress = 0

    controller = WallFollowerController()
    pid1 = PID(0.15, 0.001, 0.15, setpoint=22) # 0.3, 0.001, 0.15 work well, tune down Kp to make smoother moves
    pid2 = PID(0.15, 0.001, 0.15, setpoint=22) # 0.3, 0.001, 0.15 work well, tune down Kp to make smoother moves
    pid1.sample_time = 0.01
    pid2.sample_time = 0.01

    ego_rececar = Racecar(0, env, speed=speed1)
    ego_rececar.activate()
    env.ego_lane = 0

    from pynput.keyboard import Key, Listener

    def on_press(key):
        if key == Key.left:
            ego_rececar.switch_lane(0)
        elif key == Key.right:
            ego_rececar.switch_lane(1)
    # Collect events until released
    def listener():
        with Listener(
                on_press=on_press) as listener:
            listener.join()
    from threading import Thread
    t = Thread(target=listener)
    t.start()

    while not done:
        steer1, steer2 = get_control(pid1, env, 0), get_control(pid2, env, 1)
        control_ego = ego_rececar.update()
        obs, step_reward, done, info = env.step(np.array([control_ego, [steer2, speed2]]))
        current_lane = lane_changer.get_current_lane()
        pid1.setpoint = lane_centers[0]
        pid2.setpoint = current_lane
        
        laptime += step_reward
        if DISPLAY:
            env.render(mode='human')
        pos = env.sim.agents[1].state[0:2]
        angle = env.sim.agents[1].state[4]
        angle_pos = angle_pos_to_control(angle, pos)
        


if __name__ == '__main__':
    main()
