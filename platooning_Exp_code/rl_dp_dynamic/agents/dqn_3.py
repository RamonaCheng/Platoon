'''
@Author: WANG Maonan
@Date: 2023-03-31 10:29:09
@Description: 
@LastEditTime: 2023-03-31 16:06:51
'''
import gym
import random
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from statistics import mean

from datetime import datetime
import sumolib
import traci
import simpla
import os
import sys
from sumolib import checkBinary

import parameters_input as para

# read nominal parameters
LEARNING_RATE, MAX_MEMORY, BATCH_SIZE, GAMMA, gamma, w_1, w_2, d1, collision_time_delay, exploration_rate = para.nominal_values()

class JuctionAgent:
    def __init__(self):
        self.temp_vehicle = None
        self.temp_time = 0.0
        self.temp_vehicle_des = 0

        self.InLinks = ['link16', 'link19']
        self.InNodes = [11, 13]

        self.vehicle_count = 0
        self.state = []
        self.vehicle_container = []

    def get_veh_des(self, temp_vehicle_id):
        temp_vehicle_des = int(temp_vehicle_id[11]) + 1

        return temp_vehicle_des


    def reset(self, learning_rate, _lambda):
        self.temp_vehicle = None
        self.temp_time = 0.0
        self.temp_vehicle_des = 0
        self.vehicle_count = 0
        self.state = []
        self.vehicle_container = []


    def get_state(self, junction_vehicle, junction_time):
        if self.vehicle_count == 0:
            self.vehicle_count += 1

            # store vehicle in the container
            self.vehicle_container.append(junction_vehicle)
            
            return
        else:
            self.temp_vehicle = junction_vehicle
            self.temp_time = junction_time

            if self.temp_vehicle in self.vehicle_container:
                return
            else:
                #print('Current vehicle id: ', self.temp_vehicle)
                #print('junction 3 state: ', self.state)
                self.temp_vehicle_des = self.get_veh_des(self.temp_vehicle)
                self.state = [self.temp_vehicle_des]

                self.vehicle_container.append(self.temp_vehicle)
                if len(self.vehicle_container) > 10:
                    self.vehicle_container.pop(0)
                        
                return self.state

    def update_onestep(self, junction_vehicle):
        # return previous node
        current_link = traci.vehicle.getRoadID(self.temp_vehicle)
        previous_node = self.InNodes[self.InLinks.index(current_link)]
        # return minimum q value
        min_q = 0.0

        return previous_node, min_q