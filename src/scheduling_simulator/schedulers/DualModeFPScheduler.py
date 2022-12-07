from ..Scheduler import Scheduler

class DualModeFPScheduler(Scheduler):
    def __init__(self):
        self.__safe_mode = False

    def safe_mode(self, safe_mode):
        self.__safe_mode = safe_mode

    def schedule(self, tasks):
        task_order_normal = ['Localize', 'Opponent Localize', 'Empty', 'Controller']
        task_order_danger = ['Localize', 'Opponent Localize', 'Controller', 'Empty']

        task_order = task_order_normal
        if self.__safe_mode:
            task_order = task_order_danger

        def check_task_done(task_name):
            for task in tasks:
                if task.name == task_name:
                    return task.is_done(), task
            return True, None

        for name in task_order:
            done, task = check_task_done(name)
            if not done:
                return task
        return None
