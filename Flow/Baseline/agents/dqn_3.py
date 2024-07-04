import gym
import random
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from statistics import mean
import h5py
#import beta_calculation as bc

from datetime import datetime
import sumolib
import traci
import simpla
import os
import sys
from sumolib import checkBinary

import parameters_input as para

# read nominal parameters
LEARNING_RATE, MAX_MEMORY, BATCH_SIZE, GAMMA, EXPLORATION_DECAY, EXPLORATION_MIN, initial_exploration_rate, w_1, w_2, d1, collision_time_delay, constant_time_reduction, threshold_default_platoon = para.nominal_values()


class JuctionAgent:

    def __init__(self):
        self.lead_vehicle = None
        self.temp_vehicle = None
        self.lead_time = 0.0
        self.temp_time = 0.0
        self.lead_vehicle_des = 0
        self.temp_vehicle_des = 0
        self.lead_vehicle_link = None

        self.InLinks = ['link3']
        self.InNodes = [2]

        self.ini_sk = 0.0
        self.sk = 0.0
        self.vehicle_count = 0
        self.state = []
        self.vehicle_container = []


    def reset(self):
        self.lead_vehicle = None
        self.temp_vehicle = None
        self.lead_time = 0.0
        self.temp_time = 0.0
        self.lead_vehicle_des = 0
        self.temp_vehicle_des = 0
        self.lead_vehicle_link = None
        self.ini_sk = 0.0
        self.sk = 0.0
        self.vehicle_count = 0
        self.state = []
        self.vehicle_container = []

    def get_state(self, junction_vehicle, junction_time):
        if self.vehicle_count == 0:
            self.lead_vehicle = junction_vehicle
            self.lead_time = junction_time
            self.vehicle_count += 1

            # store vehicle in the container
            self.vehicle_container.append(junction_vehicle)
            
            return
        else:
            self.temp_vehicle = junction_vehicle
            self.temp_time = junction_time
            self.time_interval = self.temp_time - self.lead_time

            if self.temp_vehicle in self.vehicle_container:
                return
            else:
                #print('Current vehicle id: ', self.temp_vehicle)
                #print('junction 2 state: ', self.state)
                self.ini_sk = 0.0
                self.sk = self.ini_sk + self.time_interval
                self.state = [self.sk]

                self.vehicle_container.append(self.temp_vehicle)
                if len(self.vehicle_container) > 10:
                    self.vehicle_container.pop(0)
                        
                return self.state

    def update_onestep(self, junction_vehicle):
        # return previous node
        previous_node = 2
        # return minimum q value
        min_q = 0.0

        # finish the update process and change the lead vehicle and temp vehicle
        self.lead_vehicle = self.temp_vehicle
        self.lead_time = self.temp_time

        return previous_node, min_q