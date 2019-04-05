import sys
sys.path.append('..')
import numpy as np
from parameters import planner_parameters as PLAN

class msg_map:
    def __init__(self):
        self.flag_map_changed = True
        self.city_width = PLAN.city_width
        self.num_city_blocks = PLAN.num_blocks
        self.street_width = PLAN.city_width/PLAN.num_blocks * PLAN.street_width
        self.building_max_height = PLAN.building_height
        #Array of heights of buildings
        self.building_height = self.building_max_height * np.abs(np.random.randn(self.num_city_blocks, self.num_city_blocks))
        #Width of each building is the same
        self.building_width = PLAN.city_width/PLAN.num_blocks * (1- PLAN.street_width)
        #Array of north corner of the building
        self.building_north = np.zeros(self.num_city_blocks)
        self.building_east = np.zeros(self.num_city_blocks)
        for i in range(self.num_city_blocks):
            self.building_north[i] = 0.5 * self.city_width/self.num_city_blocks * (2*(i-1) + 1)
            self.building_east[i] = 0.5 * self.city_width/self.num_city_blocks * (2*(i-1) + 1)
