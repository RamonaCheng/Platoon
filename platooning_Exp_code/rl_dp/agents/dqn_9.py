'''
@Author: WANG Maonan
@Date: 2023-03-31 10:29:09
@Description: 
@LastEditTime: 2024-01-06 00:51:17
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
import calculate_d2 as dis
import theta_calculation as threshold_func
#import r_calculation as rc

# read nominal parameters
LEARNING_RATE, MAX_MEMORY, BATCH_SIZE, GAMMA, gamma, w_1, w_2, d1, collision_time_delay, exploration_rate = para.nominal_values()

class JuctionAgent:
    def __init__(self, observation_space, action_space):
        self.temp_memory = deque(maxlen=MAX_MEMORY)
        self.memory = deque(maxlen=MAX_MEMORY)
        
        self.Qmatrix = [[270.8333333333333, 270.8333333333333], [270.8333333333333, 220.83333333333334]]      # end2_link12, end2_link12, end3_link12, end3_link13

        self.lead_vehicle = None
        self.temp_vehicle = None
        self.lead_time = 0.0
        self.temp_time = 0.0
        self.lead_vehicle_des = 0
        self.temp_vehicle_des = 0
        self.lead_vehicle_link = None

        self.InLinks = ['link4', 'link6']
        self.OutLinks = ['link12', 'link13']
        self.InNodes = [4, 5]
        self.OutNodes = [10, 13]
        #self.OutLength = [10.0, 9.0]         # kilometers

        self.ini_sk = 0.0
        self.sk = 0.0
        self.vehicle_count = 0
        self.state = []
        self.vehicle_container = []
        self.time_interval_list = []
        self.arrival_rate = 0.0
        
        self.action = 12                                # 12 represents link12, 13 represents link13
        self.action_platoon = 0                         # 0 represents travel alone, 1 represents platooning
        #self.initial_value = [1.0, 25.0, -25.0]         # initial value for the threshold and C value
        self.initial_value = [1.0, 10.0, -10.0]
        self.threshold = self.initial_value[1]
        self.C = self.initial_value[2]
        self.LEARNING_RATE = LEARNING_RATE
        self._lambda = 1

    def get_veh_des(self, lead_vehicle_id, temp_vehicle_id):
        lead_vehicle_des = int(lead_vehicle_id[11]) + 1
        temp_vehicle_des = int(temp_vehicle_id[11]) + 1

        return lead_vehicle_des, temp_vehicle_des


    def reset(self, learning_rate, _lambda):
        self.temp_memory = deque(maxlen=MAX_MEMORY)
        self.memory = deque(maxlen=MAX_MEMORY)
        self.Qmatrix = [[270.8333333333333, 270.8333333333333], [270.8333333333333, 220.83333333333334]]
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
        self.action = 12
        self.action_platoon = 0                         
        #self.initial_value = [1.0, 25.0, -25.0]
        self.initial_value = [1.0, 10.0, -10.0]         
        self.threshold = self.initial_value[1]
        self.C = self.initial_value[2]
        self.LEARNING_RATE = learning_rate
        self._lambda = _lambda # lambda*exp_travel_time + (1-lambda)*this_travel_time


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
                traci.vehicle.setVia(junction_vehicle, ['link12'])
                traci.vehicle.rerouteEffort(junction_vehicle)
            
                # change the current vehicle as the leading vehicle
                self.lead_vehicle_link = 12
                self.lead_path = dis.return_path(9, temp_des, dis.return_initial_Qmatrix_Table())
            elif temp_des == 3:
                # select the shorest path
                traci.vehicle.setVia(junction_vehicle, ['link13'])
                traci.vehicle.rerouteEffort(junction_vehicle)
            
                # change the current vehicle as the leading vehicle
                self.lead_vehicle_link = 13
                self.lead_path = dis.return_path(9, temp_des, dis.return_initial_Qmatrix_Table())

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
                #print('junction 9 state: ', self.state)

                self.vehicle_container.append(self.temp_vehicle)
                if len(self.vehicle_container) > 10:
                    self.vehicle_container.pop(0)

                return self.state


    def take_action(self, state, Qmatrix_table):
        # determine the state
        current_node = 9
        sk = state[0]
        lead_des = state[1]
        temp_des = state[2]
        lead_action = state[3]
        self.norm_state = [temp_des]
        self.norm_state = np.reshape(self.norm_state, (1,1))

        # decide the routing process
        p = random.random()

        if temp_des == 3:
            if p < exploration_rate:
                self.action = random.choice([12,13])
            else:
                q_values = self.Qmatrix[1]
                if q_values[0] <= q_values[1]:
                    self.action = 12
                else:
                    self.action = 13
        elif temp_des == 2:
            self.action = 12

        # decide the platooning process
        # calculate the cruising distance
        if self.action == 12:
            temp_path = [10] + dis.return_path(10, temp_des, Qmatrix_table)
        elif self.action == 13:
            temp_path = [13] + dis.return_path(13, temp_des, Qmatrix_table)

        d2_distance, original_equiva_distance = dis.d2_distance(current_node, lead_des, temp_des, lead_action, self.action, Qmatrix_table, self.lead_path, temp_path)
        # store the leading vehicle path
        self.lead_path = temp_path

        if d2_distance == 0 or sk > int(d1/24)-2:
            self.action_platoon = 0
            # decide the deceleration process
            final_value = threshold_func.theta_c(self.arrival_rate, self.initial_value, w_1/3600.0, gamma, original_equiva_distance, 24.0, w_2)
            if abs(final_value[2]) > 50 or (final_value[1] < 0 and final_value[2] > 0):
                final_value = [1.0, 10.0, -10.0]
            self.C = final_value[2]

            self.initial_value = final_value
        else:
            # using the threshold and C strategy
            final_value = threshold_func.theta_c(self.arrival_rate, self.initial_value, w_1/3600.0, gamma, d2_distance, 24.0, w_2)
            if abs(final_value[2]) > 50 or (final_value[1] < 0 and final_value[2] > 0):
                final_value = [1.0, 10.0, -10.0]
            self.threshold = final_value[1]
            self.C = final_value[2]

            self.initial_value = final_value

            if sk > self.threshold:
                self.action_platoon = 0
            else:
                self.action_platoon = 1

        #self.action_platoon = 0
        #self.C = collision_time_delay
        
        #print('junction 9 action: ', self.action)


    def update_onestep(self):
        # return previous node
        current_link = traci.vehicle.getRoadID(self.temp_vehicle)
        previous_node = self.InNodes[self.InLinks.index(current_link)]
        # return minimum q value
        temp_des = self.state[2]

        if temp_des == 3:
            min_q = min(self.Qmatrix[1])
        elif temp_des == 2:
            min_q = self.Qmatrix[0][0]

        return previous_node, min_q


    def experience_replay(self):
        if len(self.memory) < BATCH_SIZE:
            return
        else:
            #minibatch = random.sample(self.memory, BATCH_SIZE)
            minibatch = list(self.memory)[-BATCH_SIZE:]
            for state, action, min_q, cost in minibatch:
                temp_des = state[0][0]
                Q = cost + GAMMA * min_q

                if temp_des == 3:
                    if action == 12:
                        self.Qmatrix[1][0] = self.Qmatrix[1][0] + self.LEARNING_RATE * (Q - self.Qmatrix[1][0])
                    elif action == 13:
                        self.Qmatrix[1][1] = self.Qmatrix[1][1] + self.LEARNING_RATE * (Q - self.Qmatrix[1][1])
                elif temp_des == 2:
                    self.Qmatrix[0][0] = self.Qmatrix[0][0] + self.LEARNING_RATE * (Q - self.Qmatrix[0][0])



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
            time_deduction = self.C - collision_time_delay
            #time_deduction = 0.0
            traci.vehicle.setType(self.temp_vehicle, 'non_merging')

        set_follower_speed = d1 / (d1 / 24.0 - time_deduction)
        set_follower_speed = set_follower_speed if set_follower_speed < 40 else 40 # 设置跟车的速度

        traci.vehicle.setSpeedMode(self.temp_vehicle, 0)
        traci.vehicle.setSpeed(self.temp_vehicle, set_follower_speed)

        # route choice decision
        if self.action == 12:   
            traci.vehicle.setVia(self.temp_vehicle, ['link12'])
        elif self.action == 13:
            traci.vehicle.setVia(self.temp_vehicle, ['link13'])
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

    def _f(self, x):
        """过滤探测器之后的信息

        Args:
            x: 包含两个部分，分别是 (pos, speed) 
        """
        if x[0] > 200 and x[0] < self.edge_length-d1-100:
            return x

    def add_to_memory(self, current_vehicle_id, min_q, departure_link_fuel, departure_link_time):
        # deal with the scenario for the first vehicle
        vehicle_cost = 0.0

        for item in list(self.temp_memory):
            if current_vehicle_id == item[0]:

                self.edge_id = traci.vehicle.getRoadID(current_vehicle_id)
                self.edge_length = traci.lane.getLength(f'{self.edge_id}_0')
                # 获得 vehicle 所在的车道
                veh_ids = traci.edge.getLastStepVehicleIDs(self.edge_id)
                pos_speed = [
                    (traci.vehicle.getLanePosition(_veh_id), traci.vehicle.getSpeed(_veh_id)) 
                    for _veh_id in veh_ids
                ]
                v_speeds_list = [i[1] for i in filter(self._f, pos_speed)] # get vehicle speed
                last_time_speed = 24 if len(v_speeds_list) == 0 else np.max(v_speeds_list)
                exp_vehicle_time = self.edge_length/last_time_speed # 当前进入车道的期望时间
                this_vehicle_time = departure_link_time-item[4] # 这一辆车当前的驾驶时间
                mix_vehicle_time = self._lambda*exp_vehicle_time + (1-self._lambda)*this_vehicle_time

                _fuel_cost = (departure_link_fuel-item[3])/1000.0 * w_2
                _time_cost = (departure_link_time-item[4])/3600.0 * w_1
                vehicle_cost = (departure_link_fuel-item[3])/1000.0 * w_2 + (departure_link_time-item[4])/3600.0 * w_1
                # print(f'Last Time Speed, {last_time_speed} || Exp vehicle time {exp_vehicle_time} || Vehicle time {this_vehicle_time} || Mix {mix_vehicle_time}')


                self.memory.append((item[1], item[2], min_q, mix_vehicle_time))
                self.temp_memory.remove(item)

        return vehicle_cost, _fuel_cost, _time_cost

    def return_time_matrix(self):
        return self.Qmatrix

    def return_trained_model(self):
        return self.model

    def save_model(self):
        self.model.save('Agent9.h5')
