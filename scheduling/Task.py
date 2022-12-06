class Task:
    task_counter = 0

    def __init__(self, period, duration, on_done=None, deadline_miss_callback=None, name=None):
        self.task_id = Task.task_counter
        Task.task_counter += 1
        self.name = name
        self.period = period
        self.duration = duration
        self.current_duration = 0
        self.counter = 0
        self.activation_time = 0

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

    def update(self, is_active, current_time):
        if self.counter >= self.period:
            if self.current_duration == 0:
                self.on_done()
            else:
                self.deadline_miss_callback()
            self.activate(current_time)
        if self.current_duration > 0 and is_active:
            self.current_duration -= 1
        self.counter += 1
