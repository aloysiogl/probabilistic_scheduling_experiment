class Task:
    task_counter = 0

    def __init__(self, period, duration, arrival_time=0, on_done=None, deadline_miss_callback=None, name=None):
        self.task_id = Task.task_counter
        Task.task_counter += 1
        self.name = name
        self.period = period
        self.duration = duration
        self.current_duration = 0
        self.counter = 0
        self.activation_time = 0
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
        self.current_duration = self.duration
        self.counter = 0
        self.activation_time = activation_time

    def get_deadline(self):
        return self.period+self.activation_time

    def is_done(self):
        return self.current_duration == 0

    @property
    def dmp_estimate(self):
        num = self.__missed_deadlines
        den = self.__succeeded_deadlines + self.__missed_deadlines
        if den == 0:
            return 0
        return num/den

    def update(self, is_active, current_time):
        if self.counter >= self.period:
            if self.current_duration == 0:
                self.on_done()
            else:
                self.deadline_miss_callback()
                self.__missed_deadlines += 1
            self.activate(current_time)
            return
        if self.current_duration > 0 and is_active and self.counter >= self.__arrival_time:
            self.current_duration -= 1
            if self.current_duration == 0:
                self.__succeeded_deadlines += 1
        self.counter += 1
