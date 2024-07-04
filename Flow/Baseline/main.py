import os
import sys

try:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
except KeyError:
    raise Exception("Please set the 'SUMO_HOME' environment variable.")

import gym
import random
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from statistics import mean
import h5py

from datetime import datetime
import sumolib
import traci
import simpla

from sumolib import checkBinary

import sumo_env as SumoEnv
import agents.dqn_1 as Agent1
import agents.dqn_2 as Agent2
import agents.dqn_3 as Agent3


GUI_choice = "sumo"

junction_list = ['1', '2', '3']
coordinated_junction_list = ['1', '2']


try:
    sys.path.append(os.path.join(os.path.dirname(
    __file__), '..', '..', '..', '..', "tools"))
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
    os.path.dirname(__file__), "..", "..", "..")), "tools"))
except ImportError:
    sys.exit("please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

# choose whether to use GUI or not
netconvertBinary = checkBinary('netconvert')
sumoBinary = checkBinary('sumo')

# generate the final SUMO file, include net file and vehicle file
traci.start([sumoBinary, '-c', os.path.join("Nguyen-Dupuis/Nguyen.sumocfg.xml")])

simpla.load("data/simpla.cfg.xml")
mgr=simpla._mgr
    
ENV_NAME = 'dqnsumo'
env=SumoEnv.network("Nguyen-Dupuis/newND.net.xml",GUI_choice,"Nguyen-Dupuis/Nguyen.sumocfg.xml")

# the order of the agent is named by the actual ids
agent_1 = Agent1.JuctionAgent(1, 2)
agent_2 = Agent2.JuctionAgent(1, 2)
agent_3 = Agent3.JuctionAgent()


print("---------------------------------")
print("Simulation starts")
print("---------------------------------")

episode_list = range(1)
total_cost_list = []


# begin the episode
for episode in range(1):
    # reset the q value list
    q1_list = []
    q2_list = []
    # reset the environment
    arrival_junction, arrival_vehicle, arrival_time, arrival_fuel, arrival_link_time = env.reset()
    agent_1.reset()
    agent_2.reset()
    agent_3.reset()
    # initialize total cost
    total_cost, cost1, cost2 = 0.0, 0.0, 0.0
    # begin the simulation
    for i in range(180000): # the real time in SUMO is 180000*0.5 = 90000
    #for i in range(1000):
        # the first three elements are for enviroment information, the last two elements are for cost
        arrival_junction, arrival_vehicle, arrival_time, arrival_fuel, arrival_link_time = env.observe()
        if len(arrival_junction) > 0:
            for j in range(len(arrival_junction)):
                # change the id number into our own id number
                junction_id = int(junction_list[arrival_junction[j]])
                junction_vehicle = arrival_vehicle[j]
                junction_time = arrival_time[j]
                junction_fuel = arrival_fuel[j]
                junction_link_time = arrival_link_time[j]


                if junction_id == 2:
                    state2 = agent_2.get_state(junction_vehicle, junction_time)
                    if state2 is not None:
                        agent_2.take_action(state2)
                        previous_node, min_q = agent_2.update_onestep()
                        q2_list.append([i, min_q])
                        #print('Agent 2 State: ', state2)
                        #print('Agent 2 Min Q: ', min_q)
                        if previous_node == 1:
                            #print('Agent 2 Min Q: ', min_q)
                            cost1 += agent_1.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_1.experience_replay()
                        agent_2.coordinate(junction_fuel, junction_link_time)
                                    

                # the start point id coordination
                elif junction_id == 1:
                    state1 = agent_1.get_state(junction_vehicle, junction_time)
                    if state1 is not None:
                        agent_1.take_action(state1)
                        previous_node, min_q = agent_1.update_onestep()
                        #print('Agent 1 State: ', state1)
                        #print('Agent 1 Min Q: ', min_q)
                        q1_list.append([i, min_q])
                        agent_1.coordinate(junction_fuel, junction_link_time)


                # the end point cost calculation
                elif junction_id == 3:
                    state3 = agent_3.get_state(junction_vehicle, junction_time)
                    if state3 is not None:
                        previous_node, min_q = agent_3.update_onestep(junction_vehicle)
                        #print('Agent 3 Min Q: ', min_q)
                        if previous_node == 2:
                            cost2 += agent_2.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_2.experience_replay()

        #if i > 4000 and i % 1000 == 0:
        #    agent_1.show_curve()
        #    agent_2.show_curve()


    #agent_1.save_model()
    #agent_2.save_model()
    #agent_3.save_model()

    #np.save('q1', q1_list)
    #np.save('q2', q2_list)

    cost_non_cav = env.return_cost_non_cav()
    cost_cav = env.return_cost_cav()

    total_cost = cost1 + cost2
    total_cost = total_cost * 40.0
    total_cost += cost_non_cav
    total_cost += cost_cav
    print('Episode: ', episode)
    print('total cost: ', total_cost)
    # save the final results as np file
    np.save('Episode_' + str(episode), total_cost)
    # record total cost for plot
    total_cost_list.append(total_cost)

#plt.plot(episode_list, total_cost_list)
#plt.show()





        




