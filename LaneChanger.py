import numpy as np

class LaneChanger:
    def __init__(self, initial_lane_id, lanes):
        self.current_lane_id = initial_lane_id
        self.lanes = lanes
        
    def get_current_lane(self):
        # Probability of lane change
        p = 0.001
        if np.random.rand() < p:
            self.current_lane_id = (self.current_lane_id + 1) % len(self.lanes)
        return self.lanes[self.current_lane_id]