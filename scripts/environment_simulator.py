import gym
import numpy as np
from simple_pid import PID
from pyglet.gl import GL_POINTS
from pynput.keyboard import Key, Listener

from scheduling_simulator.Racecar import Racecar

def add_circle_to_map(env, radius, scale):

    points = [np.array([np.cos(theta), np.sin(theta)])*radius*scale for theta in np.linspace(0, 2*np.pi, 100)]
    for p in points:
        env.batch.add(1, GL_POINTS, None, ('v3f/stream', [p[0], p[1], 0.]),
                            ('c3B/stream', [183, 193, 222]))


def get_render_callback():
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
    # Setting up simulation parameters
    DISPLAY = True
    env = gym.make('f110_gym:f110-v0', map='./map', map_ext='.pgm', num_agents=2, timestep=0.005)
    env.add_render_callback(get_render_callback())

    width = 4
    inner_radius = 20

    n_lanes = 2
    lane_width = width/n_lanes
    lane_centers = np.linspace(0, lane_width*(n_lanes-1), n_lanes)+lane_width/2+inner_radius
    env.lane_centers = lane_centers

    car_2_initial_angle = np.pi/6
    car_2_initial_lane = lane_centers[1]
    car_2_initial_pos = [np.cos(car_2_initial_angle)*car_2_initial_lane,
                         np.sin(car_2_initial_angle)*car_2_initial_lane, 
                         np.pi/2+car_2_initial_angle]

    speed_ego, speed_opponent = 6, 5
    
    env.reset(np.array([[lane_centers[0], 0, np.pi/2], car_2_initial_pos]))

    if DISPLAY:
        env.render()

    # Control for oppoenent vehicle
    pid2 = PID(0.15, 0.001, 0.15, setpoint=22) # 0.3, 0.001, 0.15 work well, tune down Kp to make smoother moves
    pid2.sample_time = 0.01
    current_lane_oppoenent = 1

    def get_control(pid, env, car_index): 
        pos = env.sim.agents[car_index].state[0:2]
        steer = -pid(np.linalg.norm(pos))
        return steer
    
    def get_current_lane():
        # Probability of lane change
        nonlocal current_lane_oppoenent
        p = 0.001
        if np.random.rand() < p:
            current_lane_oppoenent = (current_lane_oppoenent + 1) % len(lane_centers)
        return lane_centers[current_lane_oppoenent]

    # Control for ego vehicle
    ego_rececar = Racecar(0, env, speed=speed_ego)
    ego_rececar.activate()
    env.ego_lane = 0

    # Keyboard listener
    def on_press(key):
        if key == Key.left:
            ego_rececar.switch_lane(0)
        elif key == Key.right:
            ego_rececar.switch_lane(1)
    def listener():
        with Listener(
                on_press=on_press) as listener:
            listener.join()
    from threading import Thread
    t = Thread(target=listener, daemon=True)
    t.start()

    # Simulation loop
    current_time = 0
    while True:
        steer_opponent = get_control(pid2, env, 1)
        control_ego = ego_rececar.update(current_time)
        env.step(np.array([control_ego, [steer_opponent, speed_opponent]]))
        pid2.setpoint = get_current_lane()
        
        if DISPLAY:
            env.render(mode='human')

if __name__ == '__main__':
    main()
