'''
@Author: WANG Maonan
@Date: 2023-03-31 10:29:09
@Description: 
@LastEditTime: 2023-03-31 21:15:29
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
import r_calculation as rc

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
        self.lead_vehicle_link = None

        self.InLinks = ['link5', 'link17']
        self.OutLinks = ['link7', 'link8']
        self.InNodes = [5, 12]
        self.OutNodes = [7, 10]
        #self.OutLength = [5.0, 13.0]         # kilometers

        self.ini_sk = 0.0
        self.sk = 0.0
        self.vehicle_count = 0
        self.state = []
        self.vehicle_container = []
        self.time_interval_list = []
        self.arrival_rate = 0.0
        
        self.action = 7                                 # 7 represents link7, 8 represents link8
        self.action_platoon = 0                         # 0 represents travel alone, 1 represents platooning

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
        self.lead_vehicle_link = None

        self.ini_sk = 0.0
        self.sk = 0.0
        self.vehicle_count = 0
        self.state = []
        self.vehicle_container = []
        self.time_interval_list = []
        self.arrival_rate = 0.0

        self.action = 7
        self.action_platoon = 0


    def get_state(self, junction_vehicle, junction_time):
        if self.vehicle_count == 0:
            self.lead_vehicle = junction_vehicle
            self.lead_time = junction_time
            self.vehicle_count += 1

            self.ini_sk = 0.0

            # set the speed for the first vehicle
            traci.vehicle.setSpeedMode(junction_vehicle, 0)
            traci.vehicle.setSpeed(junction_vehicle, 24.0)

            # decide the path and change the current vehicle as the leading vehicle
            temp_des = int(junction_vehicle[11]) + 1
            if temp_des == 2:
                # select the shorest path
                traci.vehicle.setVia(junction_vehicle, ['link7'])
                traci.vehicle.rerouteEffort(junction_vehicle)
            
                # change the current vehicle as the leading vehicle
                self.lead_vehicle_link = 7
                self.lead_path = traci.simulation.findRoute('link7', 'link11', 'connected').edges
            elif temp_des == 3:
                # select the shorest path
                traci.vehicle.setVia(junction_vehicle, ['link7'])
                traci.vehicle.rerouteEffort(junction_vehicle)
            
                # change the current vehicle as the leading vehicle
                self.lead_vehicle_link = 7
                self.lead_path = traci.simulation.findRoute('link7', 'link16', 'connected').edges

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
                self.state = [self.sk, self.lead_vehicle_des, self.temp_vehicle_des, self.lead_vehicle_link]

                #print('Current vehicle id: ', self.temp_vehicle)
                #print('junction 6 state: ', self.state)

                self.vehicle_container.append(self.temp_vehicle)
                if len(self.vehicle_container) > 10:
                    self.vehicle_container.pop(0)

                return self.state

    def take_action(self, state, junction_vehicle):
        # determine the state
        current_node = 6
        sk = state[0]
        temp_des = state[2]
        lead_action = state[3]
        self.norm_state = [temp_des]
        self.norm_state = np.reshape(self.norm_state, (1,1))

        # decide the routing process
        current_link = traci.vehicle.getRoadID(junction_vehicle)
        fromEdge = current_link
        route_links = traci.vehicle.getRoute(junction_vehicle)
        toEdge = route_links[-1]
        stageResult = traci.simulation.findRoute(fromEdge, toEdge, 'connected')
        temp_path = stageResult.edges[1:-1]

        if temp_path[0] == 'link7':
            self.action = 7
        elif temp_path[0] == 'link8':
            self.action = 8

        # decide the platooning process
        # calculate the cruising distance
        if self.action != lead_action:
            d2_distance = 0
        else:
            original_time = traci.simulation.findRoute(temp_path[0], temp_path[-1], 'connected').travelTime
            if temp_path == self.lead_path:
                d2_distance = original_time
            else:
                i = 0
                while temp_path[i] == self.lead_path[i]:
                    i += 1
                # decide the split link
                split_link = temp_path[i]

                d2_distance = original_time - traci.simulation.findRoute(split_link, temp_path[-1], 'connected').travelTime

        # store the leading vehicle path
        self.lead_path = temp_path

        if d2_distance == 0 or sk > (d1/24)-3:
            self.action_platoon = 0
        else:
            # Calculate the threshold
            threshold = rc.r_calculaton(self.arrival_rate, w_1/3600.0, d2_distance)

            if self.time_interval > threshold:
                self.action_platoon = 0
            else:
                self.action_platoon = 1

        #print('junction 6 action: ', self.action)


    def update_onestep(self):
        # return previous node
        current_link = traci.vehicle.getRoadID(self.temp_vehicle)
        previous_node = self.InNodes[self.InLinks.index(current_link)]
        
        return previous_node


    def coordinate(self, detected_fuel, detected_time):
        sk = self.state[0]
        lead_des = self.state[1]
        temp_des = self.state[2]
        lead_action = self.state[3]

        # set the speed on d1
        if self.action_platoon == 1:
            time_deduction = self.sk - collision_time_delay
            traci.vehicle.setType(self.temp_vehicle, 'merging')

        else:
            time_deduction = 0.0
            traci.vehicle.setType(self.temp_vehicle, 'non_merging')

        set_follower_speed = d1 / (d1 / 24.0 - time_deduction)
        set_follower_speed = set_follower_speed if set_follower_speed < 40 else 40

        traci.vehicle.setSpeedMode(self.temp_vehicle, 0)
        traci.vehicle.setSpeed(self.temp_vehicle, set_follower_speed)

        # route choice decision
        if self.action == 7:   
            traci.vehicle.setVia(self.temp_vehicle, ['link7'])
        elif self.action == 8:
            traci.vehicle.setVia(self.temp_vehicle, ['link8'])
        traci.vehicle.rerouteEffort(self.temp_vehicle)

        #append the (state, action, fuel, time) tuple into the semi-memory
        self.add_to_semi_memory(self.temp_vehicle, self.norm_state, self.action, detected_fuel, detected_time)

        # finish the coordination process and change the lead vehicle and temp vehicle
        self.lead_vehicle = self.temp_vehicle
        self.lead_time = self.temp_time
        self.ini_sk = time_deduction
        self.lead_vehicle_link = self.action


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
