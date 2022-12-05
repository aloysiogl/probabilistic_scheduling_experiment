import numpy as np
from simple_pid import PID

from .Task import Task

class Racecar:
    def __init__(self, agent_id, env, speed=6):
        self.__agent_id = agent_id
        self.__env = env
        self.__pid_controller = PID(0.15, 0.001, 0.15)
        self.__controller_task = Task(period=4, duration=2, on_done=self.update_control_command, deadline_miss_callback=self.print_deadline_miss)
        self.__last_control_command = [0, speed]
        self.__task_list = [self.__controller_task]
        self.state = {
            'lane': 0
        }

    def activate(self):
        for task in self.__task_list:
            task.activate()

    def update_control_command(self):
        self.__pid_controller.setpoint = self.__env.lane_centers[self.state['lane']]
        pos = self.__env.sim.agents[self.__agent_id].state[0:2]
        steer = -self.__pid_controller(np.linalg.norm(pos))
        self.__last_control_command[0] = steer

    def switch_lane(self, lane):
        self.state['lane'] = lane

    def print_deadline_miss(self):
        print("Deadline miss")

    def update(self):
        # TODO the scheduler should work here
        for task in self.__task_list:
            task.update(True)
        return self.__last_control_command
