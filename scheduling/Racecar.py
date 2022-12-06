import numpy as np
from simple_pid import PID

from .Task import Task
from .tasks.DualDurationTask import DualDurationTask
from .schedulers.EDFScheduler import EDFScheduler

class Racecar:
    def __init__(self, agent_id, env, speed=6, opponent_id=1):
        self.__agent_id = agent_id
        self.__opponent_id = opponent_id
        self.__env = env

        # Tasks
        ## Control
        self.__pid_controller = PID(0.15, 0.001, 0.15)
        self.__pid_controller.sample_time = 0.005
        self.__controller_task = DualDurationTask(period=4, duration_smaller=2, duration_larger=2,
                                                  on_done=self.task_control_command, deadline_miss_callback=self.print_deadline_miss, name="Controller")
        self.__last_control_command = [0, speed]
        # Localisation
        self.__localize_task = Task(period=4, duration=1, on_done=self.task_localize, deadline_miss_callback=self.print_deadline_miss, name="Localize")
        # Opponent localisation
        self.__opponent_localize_task = Task(period=4, duration=1, on_done=self.task_opponent_localize, deadline_miss_callback=self.print_deadline_miss, name="Opponent Localize")
        self.__task_list = [self.__controller_task, self.__localize_task, self.__opponent_localize_task]
        self.__state = {
            'lane': 0,
            'opponent_lane': 0,
            'pos': np.array([0, 0]),
            'opponent_pos': np.array([0, 0])
        }

        self.__scheduler = EDFScheduler()

    def activate(self):
        for task in self.__task_list:
            task.activate()

    def task_control_command(self):
        # Decide which lane to follow
        distance_to_oppoent = np.linalg.norm(self.__state['pos'] - self.__state['opponent_pos'])
        print(f'My lane: {self.__state["lane"]}, opponent lane: {self.__state["opponent_lane"]}, distance: {distance_to_oppoent}')
        if distance_to_oppoent < 2 and self.__state['lane'] == self.__state['opponent_lane']:
            self.switch_lane((self.__state['lane'] + 1) % len(self.__env.lane_centers))

        # Make control command
        objective_lane = self.__env.lane_centers[self.__state['lane']]
        self.__pid_controller.setpoint = objective_lane
        distance_to_lane_center = abs(np.linalg.norm(self.__state['pos']) - objective_lane)
        if distance_to_lane_center > .3:
            self.__controller_task.switch_to_large_duration()
        else:
            self.__controller_task.switch_to_small_duration()
        steer = -self.__pid_controller(np.linalg.norm(self.__state['pos']))
        self.__last_control_command[0] = steer

    def task_opponent_localize(self):
        self.__state['opponent_pos'] = self.__env.sim.agents[self.__opponent_id].state[0:2]
        opponent_radius = np.linalg.norm(self.__state['opponent_pos'])
        self.__state['opponent_lane'] = np.argmin(np.abs(self.__env.lane_centers - opponent_radius))

    def task_localize(self):
        self.__state['pos'] = self.__env.sim.agents[self.__agent_id].state[0:2]

    def switch_lane(self, lane):
        self.__state['lane'] = lane

    def print_deadline_miss(self):
        # print("Deadline miss")
        pass

    def update(self, current_time):
        # TODO the scheduler should work here
        task_to_execute = self.__scheduler.schedule(self.__task_list)
        for task in self.__task_list:
            if task_to_execute and task_to_execute.get_id() == task.get_id():
                task.update(True, current_time)
            else: task.update(False, current_time)
        return self.__last_control_command
