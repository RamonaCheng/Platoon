import os
import sys
try:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
except KeyError:
    raise Exception("Please set the 'SUMO_HOME' environment variable.")

import traci
import simpla
import sumolib

import gym
import random
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from statistics import mean
import h5py
# import beta_calculation as bc


from datetime import datetime
from sumolib import checkBinary

import parameters_input as para

# read nominal parameters
LEARNING_RATE, MAX_MEMORY, BATCH_SIZE, GAMMA, EXPLORATION_DECAY, EXPLORATION_MIN, initial_exploration_rate, w_1, w_2, d1, collision_time_delay, constant_time_reduction, threshold_default_platoon = para.nominal_values()

#@TODO fix typo ;-;
class JuctionAgent:

    def __init__(self, observation_space, action_space):

        self.observation_space = observation_space
        self.action_space = action_space

        self.temp_memory = deque(maxlen=MAX_MEMORY)
        self.memory = deque(maxlen=MAX_MEMORY)
        self.memory_0 = deque(maxlen=MAX_MEMORY)
        self.memory_1 = deque(maxlen=MAX_MEMORY)

        self.beta_1 = [0.0]*6

        self.min_cost = 0.0

        self.lead_vehicle = None
        self.temp_vehicle = None
        self.lead_time = 0.0
        self.temp_time = 0.0
        self.lead_vehicle_des = 0
        self.temp_vehicle_des = 0
        # self.lead_vehicle_link = None

        self.InLinks = ['link1', 'link7']
        self.OutLinks = ['link2']

        self.ini_sk = 0.0
        self.sk = 0.0
        self.vehicle_count = 0
        self.state = []
        self.vehicle_container = []

        self.action = 2  # 0 represents not catch up, 1 reprents catch up, 2 for initial value

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
        self.action = 2
        self.min_cost = 0.0

    def get_state(self, junction_vehicle, junction_time):
        if self.vehicle_count == 0:
            self.lead_vehicle = junction_vehicle
            self.lead_time = junction_time
            self.vehicle_count += 1

            self.ini_sk = 0.0

            traci.vehicle.setSpeedMode(self.lead_vehicle, 0)
            traci.vehicle.setSpeed(self.lead_vehicle, 24.0)

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

                self.state = [self.sk]

                # print('Current vehicle id: ', self.temp_vehicle)
                # print('junction 1 state: ', self.state)

                self.vehicle_container.append(self.temp_vehicle)
                if len(self.vehicle_container) > 10:
                    self.vehicle_container.pop(0)

                return self.state

    def take_action(self, state):
        sk = state[0]
        sk_norm = state[0] / 50.0
        self.norm_state = [sk_norm]

        if sk > 40.0:
            self.action = 0
        else:
            if len(self.memory) < 40:
                self.action = random.randrange(0, self.action_space)
            else:
                # calculate estimated cost for available actions
                # the case when the number of not catch up vehicles is less than 5, we use the estimated value via equations
                if len(self.memory_0) < 5:
                    vehicle_speed = 24.0                                  # m/s
                    fuel_rate = 3.51 * \
                        (10 ** (-4)) * (vehicle_speed ** 3) + \
                        0.407 * vehicle_speed  # ml/s
                    travel_distance = (1 + 30 + 1 + 30) * 1000.0          # m
                    travel_time = travel_distance / vehicle_speed         # s
                    fuel_consumption = fuel_rate * travel_time / 1000.0   # L

                    cost_not_catch_up = travel_time * w_1 / 3600.0 + fuel_consumption * w_2
                    cost_not_catch_up = cost_not_catch_up / 40.0

                # the case when the number of not catch up vehicles is more than 5, we use the simulated average values
                else:
                    cost_not_catch_up_list = []
                    not_catch_minibatch = list(self.memory_0)[-5:]
                    for state, action, min_q, cost in not_catch_minibatch:
                        Q = cost + GAMMA * min_q
                        cost_not_catch_up_list.append(Q)

                    cost_not_catch_up = np.mean(cost_not_catch_up_list)

                cost_catch_up = self.beta_1[0] * sk_norm ** 0 + self.beta_1[1] * sk_norm ** 1 + self.beta_1[2] * \
                    sk_norm ** 2 + self.beta_1[3] * sk_norm ** 3 + \
                    self.beta_1[4] * sk_norm ** 4 + \
                    self.beta_1[5] * sk_norm ** 5

                # select the optimal action
                if cost_catch_up > cost_not_catch_up:
                    self.action = 0
                else:
                    self.action = 1

        # print('junction 1 action: ', self.action)
        return self.action

    def update_onestep(self):
        # return previous node
        previous_node = 0
        # return minimum cost
        if self.action == 0:
            # calculate estimated cost for available actions
            # the case when the number of not catch up vehicles is less than 5, we use the estimated value via equations
            if len(self.memory_0) < 5:
                vehicle_speed = 24.0                                  # m/s
                fuel_rate = 3.51 * \
                    (10 ** (-4)) * (vehicle_speed ** 3) + \
                    0.407 * vehicle_speed  # ml/s
                travel_distance = (1 + 30 + 1 + 30) * 1000.0          # m
                travel_time = travel_distance / vehicle_speed         # s
                fuel_consumption = fuel_rate * travel_time / 1000.0   # L

                cost_not_catch_up = travel_time * w_1 / 3600.0 + fuel_consumption * w_2
                cost_not_catch_up = cost_not_catch_up / 40.0

                min_q = cost_not_catch_up

            # the case when the number of not catch up vehicles is more than 5, we use the simulated average values
            else:
                cost_not_catch_up_list = []
                not_catch_minibatch = list(self.memory_0)[-5:]
                for state, action, min_q, cost in not_catch_minibatch:
                    Q = cost + GAMMA * min_q
                    cost_not_catch_up_list.append(Q)
                min_q = np.mean(cost_not_catch_up_list)

        else:
            sk_norm = self.norm_state[0]
            cost_catch_up = self.beta_1[0] * sk_norm ** 0 + self.beta_1[1] * sk_norm ** 1 + self.beta_1[2] * \
                sk_norm ** 2 + self.beta_1[3] * sk_norm ** 3 + \
                self.beta_1[4] * sk_norm ** 4 + self.beta_1[5] * sk_norm ** 5

            min_q = cost_catch_up

        return previous_node, min_q

    def experience_replay(self):
        last_vehicle_memory = list(self.memory)[-1:]

        # print('agent 1: ', last_vehicle_memory)

        action = last_vehicle_memory[0][1]

        if action == 0:
            return
        else:
            if len(self.memory_1) < 30:
                return
            else:
                sk_list = []
                cost_list = []

                minibatch = list(self.memory_1)[-30:]
                for state, action, min_q, cost in minibatch:
                    sk_list.append(state[0])
                    Q = cost + GAMMA * min_q
                    cost_list.append(Q)

                self.beta_1 = bc.polynomial_para(sk_list, cost_list, 5)

    def coordinate(self, detected_fuel, detected_time):
        # the catch up mode
        if self.action == 1:
            # set the speed
            time_deduction = self.sk - collision_time_delay
            set_follower_speed = d1 / (d1 / 24.0 - time_deduction)

            traci.vehicle.setSpeedMode(self.temp_vehicle, 0)
            traci.vehicle.setSpeed(self.temp_vehicle, set_follower_speed)

            # define the vehicle type
            traci.vehicle.setType(self.temp_vehicle, 'merging')

            # define the sk base
            self.ini_sk = self.sk - collision_time_delay

        # action is 0
        else:
            # set the speed
            traci.vehicle.setSpeedMode(self.temp_vehicle, 0)
            traci.vehicle.setSpeed(self.temp_vehicle, 24.0)

            # define the vehicle type
            traci.vehicle.setType(self.temp_vehicle, 'non_merging')

            # define the sk base
            self.ini_sk = 0.0

        # append the (state, action, fuel, time) tuple into the semi-memory
        self.add_to_semi_memory(
            self.temp_vehicle, self.norm_state, self.action, detected_fuel, detected_time)

        # finish the coordination process and change the lead vehicle and temp vehicle
        self.lead_vehicle = self.temp_vehicle
        self.lead_time = self.temp_time

    def add_to_semi_memory(self, temp_vehicle_id, state, action, arrival_link_fuel, arrival_link_time):
        self.temp_memory.append(
            (temp_vehicle_id, state, action, arrival_link_fuel, arrival_link_time))

    def add_to_memory(self, current_vehicle_id, min_q, departure_link_fuel, departure_link_time):
        # deal with the scenario for the first vehicle
        vehicle_cost = 0.0

        for item in list(self.temp_memory):
            if current_vehicle_id == item[0]:

                vehicle_cost = (
                    departure_link_fuel-item[3])/1000.0 * w_2 + (departure_link_time-item[4])/3600.0 * w_1
                vehicle_cost = vehicle_cost / 40.0

                self.memory.append((item[1], item[2], min_q, vehicle_cost))
                self.temp_memory.remove(item)

                if item[2] == 0:
                    self.memory_0.append(
                        (item[1], item[2], min_q, vehicle_cost))
                else:
                    self.memory_1.append(
                        (item[1], item[2], min_q, vehicle_cost))

        return vehicle_cost

    def show_curve(self):

        sk_list_test = np.linspace(0.0, 40.0, num=1000)
        # record the q values
        Q_0 = []
        Q_1 = []

        state_norm = 50.0
        cost_norm = 40.0

        if len(self.memory_0) < 5:
            vehicle_speed = 24.0                                  # m/s
            fuel_rate = 3.51 * (10 ** (-4)) * (vehicle_speed **
                                               3) + 0.407 * vehicle_speed  # ml/s
            travel_distance = (1 + 30 + 1 + 30) * 1000.0          # m
            travel_time = travel_distance / vehicle_speed         # s
            fuel_consumption = fuel_rate * travel_time / 1000.0   # L

            cost_not_catch_up = travel_time * w_1 / 3600.0 + fuel_consumption * w_2
            cost_not_catch_up = cost_not_catch_up / 40.0

            total_cost_not_catching = cost_not_catch_up

        # the case when the number of not catch up vehicles is more than 5, we use the simulated average values
        else:
            cost_not_catch_up_list = []
            not_catch_minibatch = list(self.memory_0)[-5:]
            for state, action, min_q, cost in not_catch_minibatch:
                Q = cost + GAMMA * min_q
                cost_not_catch_up_list.append(Q)

            total_cost_not_catching = np.mean(cost_not_catch_up_list)

        for sk_test in sk_list_test:

            sk_test = sk_test/state_norm
            # print(tf.constant(model(state)[0][0][0]))
            Q_0.append(total_cost_not_catching * cost_norm)
            estimated_cost = self.beta_1[0] * sk_test ** 0 + self.beta_1[1] * sk_test ** 1 + self.beta_1[2] * \
                sk_test ** 2 + self.beta_1[3] * sk_test ** 3 + \
                self.beta_1[4] * sk_test ** 4 + self.beta_1[5] * sk_test ** 5
            Q_1.append(estimated_cost * cost_norm)

        plt.scatter(sk_list_test, Q_1, label='Q_1')
        plt.scatter(sk_list_test, Q_0, label='Q_0')

        plt.legend(loc='best', fontsize=23)
        plt.show()
