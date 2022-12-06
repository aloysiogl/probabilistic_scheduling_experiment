from ..Task import Task

class DualDurationTask(Task):
    def __init__(self, period, duration_smaller, duration_larger, on_done=None, deadline_miss_callback=None, name=None):
        super().__init__(period, duration_smaller, on_done, deadline_miss_callback, name)
        self.__duration_larger = duration_larger
        self.__duration_smaller = duration_smaller

    def switch_to_small_duration(self):
        self.duration = self.__duration_smaller

    def switch_to_large_duration(self):
        self.duration = self.__duration_larger
    