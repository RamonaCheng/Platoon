'''
@Author: WANG Maonan
@Date: 2023-03-31 10:29:09
@Description: 
@LastEditTime: 2023-03-31 21:32:13
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
import theta_calculation as threshold_func

# read nominal parameters
LEARNING_RATE, MAX_MEMORY, BATCH_SIZE, GAMMA, gamma, w_1, w_2, d1, collision_time_delay = para.nominal_values()

class JuctionAgent:
    def __init__(self, observation_space, action_space):
        self.temp_memory = deque(maxlen=MAX_MEMORY)

        self.lead_vehicle = None
        self.temp_vehicle = None
        self.lead_time = 0.0
        self.temp_time = 0.0
        self.lead_vehicle_des = 0
        self.temp_vehicle_des = 0

        self.InLinks = ['link10', 'link14']
        self.OutLinks = ['link15', 'link16']
        self.InNodes = [7, 10]
        self.OutNodes = [2, 3]
        #self.OutLength = [9.0, 8.0]         # kilometers

        self.ini_sk = 0.0
        self.sk = 0.0
        self.vehicle_count = 0
        self.state = []
        self.vehicle_container = []
        self.time_interval_list = []
        self.arrival_rate = 0.0
        
        self.action = 15                                # 15 represents link15, 16 represents link16
        self.action_platoon = 0                         # 0 represents travel alone, 1 represents platooning
        #self.initial_value = [1.0, 25.0, -25.0]         # initial value for the threshold and C value
        self.initial_value = [0.3, 8.0, -8.0]
        self.threshold = self.initial_value[1]
        self.C = self.initial_value[2]


    def get_veh_des(self, lead_vehicle_id, temp_vehicle_id):
        lead_vehicle_des = int(lead_vehicle_id[11]) + 1
        temp_vehicle_des = int(temp_vehicle_id[11]) + 1

        return lead_vehicle_des, temp_vehicle_des

    def reset(self):
        self.temp_memory = deque(maxlen=MAX_MEMORY)
        self.lead_vehicle = None
        self.temp_vehicle = None
        self.lead_time = 0.0
        self.temp_time = 0.0
        self.lead_vehicle_des = 0
        self.temp_vehicle_des = 0

        self.ini_sk = 0.0
        self.sk = 0.0
        self.vehicle_count = 0
        self.state = []
        self.vehicle_container = []
        self.time_interval_list = []
        self.arrival_rate = 0.0

        self.action = 15
        self.action_platoon = 0
        #self.initial_value = [1.0, 25.0, -25.0]         # initial value for the threshold and C value
        self.initial_value = [0.3, 8.0, -8.0]
        self.threshold = self.initial_value[1]
        self.C = self.initial_value[2]                         


    def get_state(self, junction_vehicle, junction_time):
        if self.vehicle_count == 0:
            self.lead_vehicle = junction_vehicle
            self.lead_time = junction_time
            self.vehicle_count += 1

            self.ini_sk = 0.0

            # set the speed for the first vehicle
            traci.vehicle.setSpeedMode(junction_vehicle, 0)
            traci.vehicle.setSpeed(junction_vehicle, 24.0)

            '''
            # change the current vehicle as the leading vehicle
            temp_des = int(junction_vehicle[11]) + 1
            if temp_des == 2:
                self.lead_path = ['link15']
            elif temp_des == 3:
                self.lead_path = ['link16']
            '''

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
                self.sk = self.ini_sk + self.time_interval

                # estimate the arrival rate
                if len(self.time_interval_list) > 50:
                    self.time_interval_list.pop(0)
                self.time_interval_list.append(self.time_interval)

                gamma_estimator = 0.9
                estimated_arrval_rate = 0
                gamma_index = 0
                for i_pop_list in range(len(self.time_interval_list)):
                    estimated_arrval_rate += self.time_interval_list[len(self.time_interval_list) - 1 - i_pop_list] * (gamma_estimator ** gamma_index)
                    gamma_index += 1

                estimated_arrval_rate = estimated_arrval_rate * (1.0 - gamma_estimator)
                estimated_arrval_rate = 1.0 / estimated_arrval_rate if estimated_arrval_rate !=0 else 10
                self.arrival_rate = estimated_arrval_rate

                # get the vehicle state
                self.lead_vehicle_des, self.temp_vehicle_des = self.get_veh_des(self.lead_vehicle, self.temp_vehicle)
                self.state = [self.sk, self.lead_vehicle_des, self.temp_vehicle_des]

                #print('Current vehicle id: ', self.temp_vehicle)
                #print('junction 11 state: ', self.state)

                self.vehicle_container.append(self.temp_vehicle)
                if len(self.vehicle_container) > 10:
                    self.vehicle_container.pop(0)

                return self.state

    def take_action(self, state, junction_vehicle):
        # determine the state
        current_node = 11
        sk = state[0]
        lead_des = state[1]
        temp_des = state[2]
        self.norm_state = [temp_des]
        self.norm_state = np.reshape(self.norm_state, (1,1))

        # decide the leading vehicle action
        if lead_des == 2:
            lead_action = 15
        elif lead_des == 3:
            lead_action = 16
        
        # decide the routing process
        if temp_des == 2:
            self.action = 15
        elif temp_des == 3:
            self.action = 16

        # decide the platooning process
        # calculate the cruising distance
        if temp_des != lead_des:
            d2_distance = 0

            if temp_des == 2:
                original_time = 9.0 * 1000.0              #link 15 distance
            elif temp_des == 3:
                original_time = 8.0 * 1000.0              #link 16 distance   
        else:
            if temp_des == 2:
                d2_distance = 9.0 * 1000.0                #link 15 distance
            elif temp_des == 3:
                d2_distance = 8.0 * 1000.0                #link 16 distance

            original_time = d2_distance           


        if d2_distance == 0 or sk > (d1/24)-3:
            self.action_platoon = 0
            # decide the deceleration process
            final_value = threshold_func.theta_c(self.arrival_rate, self.initial_value, w_1/3600.0, gamma, original_time, 24.0, w_2)
            if abs(final_value[2]) > 50 or (final_value[1] < 0 and final_value[2] > 0):
                final_value = [0.3, 8.0, -8.0]
            self.C = final_value[2]

            self.initial_value = final_value
        else:
            # using the threshold and C strategy
            final_value = threshold_func.theta_c(self.arrival_rate, self.initial_value, w_1/3600.0, gamma, d2_distance, 24.0, w_2)
            if abs(final_value[2]) > 50:
                final_value = [0.3, 8.0, -8.0]
            #print('final_value: ', final_value)
            self.threshold = final_value[1]
            self.C = final_value[2]

            self.initial_value = final_value

            if sk > self.threshold:
                self.action_platoon = 0
            else:
                self.action_platoon = 1

        #print('junction 11 action: ', self.action)

    def update_onestep(self):
        # return previous node
        current_link = traci.vehicle.getRoadID(self.temp_vehicle)
        previous_node = self.InNodes[self.InLinks.index(current_link)]

        return previous_node


    def coordinate(self, detected_fuel, detected_time):
        sk = self.state[0]
        lead_des = self.state[1]
        temp_des = self.state[2]

        # set the speed on d1
        if self.action_platoon == 1:
            time_deduction = self.sk - collision_time_delay
            traci.vehicle.setType(self.temp_vehicle, 'merging')

        else:
            time_deduction = self.C - collision_time_delay
            traci.vehicle.setType(self.temp_vehicle, 'non_merging')

        set_follower_speed = d1 / (d1 / 24.0 - time_deduction)
        set_follower_speed = set_follower_speed if set_follower_speed < 40 else 40

        traci.vehicle.setSpeedMode(self.temp_vehicle, 0)
        traci.vehicle.setSpeed(self.temp_vehicle, set_follower_speed)

        #append the (state, action, fuel, time) tuple into the semi-memory
        self.add_to_semi_memory(self.temp_vehicle, self.norm_state, self.action, detected_fuel, detected_time)

        # finish the coordination process and change the lead vehicle and temp vehicle
        self.lead_vehicle = self.temp_vehicle
        self.lead_time = self.temp_time
        self.ini_sk = time_deduction


    def add_to_semi_memory(self, temp_vehicle_id, state, action, arrival_link_fuel, arrival_link_time):
        self.temp_memory.append((temp_vehicle_id, state, action, arrival_link_fuel, arrival_link_time))

    def add_to_memory(self, current_vehicle_id, departure_link_fuel, departure_link_time):
        # deal with the scenario for the first vehicle
        vehicle_cost = 0.0

        for item in list(self.temp_memory):
            if current_vehicle_id == item[0]:

                vehicle_cost = (departure_link_fuel-item[3])/1000.0 * w_2 + (departure_link_time-item[4])/3600.0 * w_1

                self.temp_memory.remove(item)

        return vehicle_cost
