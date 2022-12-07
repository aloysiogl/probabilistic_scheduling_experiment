class Task:
    task_counter = 0

    def __init__(self, period, duration, arrival_time=0, on_done=None, deadline_miss_callback=None, name=None):
        self.task_id = Task.task_counter
        Task.task_counter += 1
        self.__name = name
        self.__period = period
        self.__duration = duration
        self.__current_duration = 0
        self.__counter = 0
        self.__activation_time = 0
        self.__succeeded_deadlines = 0
        self.__missed_deadlines = 0
        self.__arrival_time = arrival_time

        self.on_done = on_done
        self.deadline_miss_callback = deadline_miss_callback
    
    @property
    def name(self):
        return self.__name

    @name.setter
    def name(self, name):
        self.__name = name

    def get_id(self):
        return self.task_id

    def activate(self, activation_time=0):
        self.__current_duration = self.__duration
        self.__counter = 0
        self.__activation_time = activation_time

    def get_deadline(self):
        return self.__period+self.__activation_time

    def is_done(self):
        return self.__current_duration == 0

    @property
    def dmp_estimate(self):
        num = self.__missed_deadlines
        den = self.__succeeded_deadlines + self.__missed_deadlines
        if den == 0:
            return 0
        return num/den

    def update(self, is_active, current_time):
        if self.__counter >= self.__period:
            if self.__current_duration == 0:
                self.on_done()
            else:
                self.deadline_miss_callback()
                self.__missed_deadlines += 1
            self.activate(current_time)
            return
        if self.__current_duration > 0 and is_active and self.__counter >= self.__arrival_time:
            self.__current_duration -= 1
            if self.__current_duration == 0:
                self.__succeeded_deadlines += 1
        self.__counter += 1
