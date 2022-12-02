from locale import normalize

import time

import gym
import numpy as np
import shapely.geometry as shp
from simple_pid import PID

from pyglet.gl import GL_POINTS

from trajectory_predictor.utils.SplineOptimizer import SplineOptimizer
from trajectory_predictor.controller.WallFollowerController import WallFollowerController
from control import angle_pos_to_control
from LaneChanger import LaneChanger

###############################################
# Generates a trajectory using wall following #
###############################################

def render_callback(env_renderer):
        # custom extra drawing function

        e = env_renderer

        # update camera to follow car
        x = e.cars[0].vertices[::2]
        y = e.cars[0].vertices[1::2]
        top, bottom, left, right = max(y), min(y), min(x), max(x)
        e.score_label.x = left
        e.score_label.y = top - 700
        e.left = left - 800
        e.right = right + 800
        e.top = top + 800
        e.bottom = bottom - 800

        # track = np.loadtxt('../track_generator/centerline/map0.csv', delimiter=',')
        # for i in range(track.shape[0]):
        #     b = e.batch.add(1, GL_POINTS, None, ('v3f/stream', [track[i, 0]*50, track[i, 1]*50, 0.]),
        #                         ('c3B/stream', [183, 193, 222]))
                # self.drawn_waypoints.append(b)
            # else:
            #     pass
                # self.drawn_waypoints[i].vertices = [scaled_points[i, 0], scaled_points[i, 1], 0.]

def main():
    N_LAPS = 1
    DISPLAY = True
    env = gym.make('f110_gym:f110-v0', map='./map', map_ext='.pgm', num_agents=1, timestep=0.01)
    env.add_render_callback(render_callback)
    # Rad csv file with numpy
    width = 4
    inner_radius = 20
    initial_x = inner_radius + width/2

    obs, step_reward, done, info = env.reset(np.array([[initial_x, 0, np.pi/2]]))

    n_lanes = 2
    lane_width = width/n_lanes
    lane_centers = np.linspace(0, lane_width*(n_lanes-1), n_lanes)+lane_width/2+inner_radius
    lane_changer = LaneChanger(0, lane_centers)

    if DISPLAY:
        env.render()

    laptime = 0.0
    start = time.time()
    speed, steer = 5,0
    progress = 0
    history = []

    controller = WallFollowerController()
    pid = PID(0.15, 0.001, 0.15, setpoint=22) # 0.3, 0.001, 0.15 work well, tune down Kp to make smoother moves
    pid.sample_time = 0.01



    while not done:
        obs, step_reward, done, info = env.step(np.array([[steer, speed]]))
        current_lane = lane_changer.get_current_lane()
        pid.setpoint = current_lane
        
        # if obs['lap_counts'][0] >= 1:
        #     pid.setpoint = 23
        
        # Control
        steer, speed = controller.get_control(obs)
        laptime += step_reward
        if DISPLAY:
            env.render(mode='human')
        pos = env.sim.agents[0].state[0:2]
        angle = env.sim.agents[0].state[4]
        angle_pos = angle_pos_to_control(angle, pos)
        steer = -pid(np.linalg.norm(pos))
        print(f'pos: {pos}, angle: {angle}, angle_pos: {angle_pos}, norm: {np.linalg.norm(pos)}, current_lane: {current_lane}, steer: {steer}')
        
    print('Sim elapsed time:', laptime, 'Real elapsed time:', time.time()-start)
    np.save('history.npy', np.array(history))

if __name__ == '__main__':
    main()
