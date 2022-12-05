class Task:
    def __init__(self, period, duration, on_done=None, deadline_miss_callback=None):
        self.period = period
        self.duration = duration
        self.current_duration = 0
        self.counter = 0

        self.on_done = on_done
        self.deadline_miss_callback = deadline_miss_callback

    def activate(self):
        self.current_duration = self.duration
        self.counter = 0

    def update(self, is_active):
        if self.counter >= self.period:
            if self.current_duration == 0:
                self.on_done()
            else:
                self.deadline_miss_callback()
            self.activate()
        if self.current_duration > 0 and is_active:
            self.current_duration -= 1
        self.counter += 1
