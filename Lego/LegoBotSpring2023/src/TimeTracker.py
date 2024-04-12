import time

class TimeTracker:

    def __init__(self):
        self.start_time = time.time()
    
    def get_current_time(self):
        end_time = time.time()
        time_elapsed = end_time - self.start_time
        return time_elapsed