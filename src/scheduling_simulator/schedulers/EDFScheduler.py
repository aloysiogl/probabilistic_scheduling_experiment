from ..Scheduler import Scheduler

class EDFScheduler(Scheduler):
    def __init__(self):
        pass
    
    def schedule(self, tasks):
        tasks.sort(key=lambda task: task.get_deadline())
        for task in tasks:
            if not task.is_done():
                print(f'Scheduling task {task.name} with deadline {task.get_deadline()}')
                return task
            else:
                # print(f'Task {task.name} is done')
                pass
        return None
