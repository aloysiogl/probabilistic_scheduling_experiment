from abc import abstractmethod


class Scheduler:
    def __init__(self):
        pass
    
    @abstractmethod
    def schedule(self, tasks):
        pass
