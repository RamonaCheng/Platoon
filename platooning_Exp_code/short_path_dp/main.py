'''
@Author: WANG Maonan
@Date: 2023-03-31 10:29:09
@Description: 
@LastEditTime: 2023-04-06 21:55:09
'''
import argparse
import numpy as np
import os
import sys

try:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
except KeyError:
    raise Exception("Please set the 'SUMO_HOME' environment variable.")

import traci
import simpla

from sumolib import checkBinary

import sumo_env as SumoEnv
import agents.dqn_1 as Agent1
import agents.dqn_2 as Agent2
import agents.dqn_3 as Agent3
import agents.dqn_4 as Agent4
import agents.dqn_5 as Agent5
import agents.dqn_6 as Agent6
import agents.dqn_7 as Agent7
import agents.dqn_8 as Agent8
import agents.dqn_9 as Agent9
import agents.dqn_10 as Agent10
import agents.dqn_11 as Agent11
import agents.dqn_12 as Agent12
import agents.dqn_13 as Agent13


junction_list = ['2', '3', '1', '5', '7', '11', '8', '9', '10', '13', '12', '6', '4']
coordinated_junction_list = ['1', '5', '7', '11', '8', '9', '10', '13', '12', '6', '4']

parser = argparse.ArgumentParser()
parser.add_argument('--jam_density', type=float, default=0.005)
parser.add_argument('--sumo_gui', default=False, action='store_true') # 默认不使用 sumo-gui
parser.add_argument('--route', type=str, default='N.sumocfg.xml')
args = parser.parse_args()

if args.sumo_gui:
    GUI_choice = "sumo-gui"
else:
    GUI_choice = "sumo"


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

traci.start(
    [
        sumoBinary, 
        '-c', "Nguyen-Dupuis/Nguyen.sumocfg.xml",
        '-r', f"Nguyen-Dupuis/{args.route}",
    ]
)


simpla.load("data/simpla.cfg.xml")
mgr=simpla._mgr
    
ENV_NAME = 'dqnsumo'
env=SumoEnv.network(
    "Nguyen-Dupuis/newND.net.xml", 
    GUI_choice, 
    sumocfgPath="Nguyen-Dupuis/Nguyen.sumocfg.xml", 
    routePath=f"Nguyen-Dupuis/{args.route}",
    jam_density=args.jam_density,
)

# the order of the agent is named by the actual ids
agent_1 = Agent1.JuctionAgent(4, 3)
agent_2 = Agent2.JuctionAgent()
agent_3 = Agent3.JuctionAgent()
agent_4 = Agent4.JuctionAgent(4, 3)
agent_5 = Agent5.JuctionAgent(4, 3)
agent_6 = Agent6.JuctionAgent(4, 3)
agent_7 = Agent7.JuctionAgent(4, 3)
agent_8 = Agent8.JuctionAgent(1, 2)
agent_9 = Agent9.JuctionAgent(4, 3)
agent_10 = Agent10.JuctionAgent(3, 2)
agent_11 = Agent11.JuctionAgent(3, 2)
agent_12 = Agent12.JuctionAgent(4, 3)
agent_13 = Agent13.JuctionAgent(1, 2)


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
    q3_list = []
    q4_list = []
    q5_list = []
    q6_list = []
    q7_list = []
    q8_list = []
    q9_list = []
    q10_list = []
    q11_list = []
    q12_list = []
    q13_list = []
    # reset the environment
    arrival_junction, arrival_vehicle, arrival_time, arrival_fuel, arrival_link_time = env.reset()
    agent_1.reset()
    agent_2.reset()
    agent_3.reset()
    agent_4.reset()
    agent_5.reset()
    agent_6.reset()
    agent_7.reset()
    agent_8.reset()
    agent_9.reset()
    agent_10.reset()
    agent_11.reset()
    agent_12.reset()
    agent_13.reset()
    # initialize total cost
    total_cost, cost1, cost4, cost5, cost6, cost7, cost8, cost9, cost10, cost11, cost12, cost13 = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    # begin the simulation
    while traci.simulation.getMinExpectedNumber()>0:
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

                #print(junction_id, junction_vehicle, junction_time, junction_fuel, junction_link_time)


                if junction_id == 5:
                    state5 = agent_5.get_state(junction_vehicle, junction_time)
                    if state5 is not None:
                        agent_5.take_action(state5, junction_vehicle)
                        previous_node = agent_5.update_onestep()
                        if previous_node == 1:
                            cost1 += agent_1.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)
                        elif previous_node == 4:
                            cost4 += agent_4.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)
                        agent_5.coordinate(junction_fuel, junction_link_time)
                
                elif junction_id == 6:
                    state6 = agent_6.get_state(junction_vehicle, junction_time)
                    if state6 is not None:
                        agent_6.take_action(state6, junction_vehicle)
                        previous_node = agent_6.update_onestep()
                        if previous_node == 5:
                            cost5 += agent_5.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)
                        elif previous_node == 12:
                            cost12 += agent_12.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)
                        agent_6.coordinate(junction_fuel, junction_link_time)

                elif junction_id == 7:
                    state7 = agent_7.get_state(junction_vehicle, junction_time)
                    if state7 is not None:
                        agent_7.take_action(state7, junction_vehicle)
                        previous_node = agent_7.update_onestep()
                        if previous_node == 6:
                            cost6 += agent_6.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)
                        agent_7.coordinate(junction_fuel, junction_link_time)

                elif junction_id == 8:
                    state8 = agent_8.get_state(junction_vehicle, junction_time)
                    #print('State 8: ', state8)
                    if state8 is not None:
                        agent_8.take_action(state8, junction_vehicle)
                        previous_node = agent_8.update_onestep()
                        if previous_node == 7:
                            cost7 += agent_7.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)
                        elif previous_node == 12:
                            cost12 += agent_12.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)
                        agent_8.coordinate(junction_fuel, junction_link_time)

                elif junction_id == 9:
                    state9 = agent_9.get_state(junction_vehicle, junction_time)
                    if state9 is not None:
                        agent_9.take_action(state9, junction_vehicle)
                        previous_node = agent_9.update_onestep()
                        if previous_node == 4:
                            cost4 += agent_4.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)
                        elif previous_node == 5:
                            cost5 += agent_5.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)
                        agent_9.coordinate(junction_fuel, junction_link_time)

                elif junction_id == 10:
                    state10 = agent_10.get_state(junction_vehicle, junction_time)
                    if state10 is not None:
                        agent_10.take_action(state10, junction_vehicle)
                        previous_node = agent_10.update_onestep()
                        if previous_node == 6:
                            cost6 += agent_6.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)
                        elif previous_node == 9:
                            cost9 += agent_9.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)
                        agent_10.coordinate(junction_fuel, junction_link_time)

                elif junction_id == 11:
                    state11 = agent_11.get_state(junction_vehicle, junction_time)
                    if state11 is not None:
                        agent_11.take_action(state11, junction_vehicle)
                        previous_node = agent_11.update_onestep()
                        if previous_node == 7:
                            cost7 += agent_7.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)
                        elif previous_node == 10:
                            cost10 += agent_10.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)
                        agent_11.coordinate(junction_fuel, junction_link_time)

                elif junction_id == 12:
                    state12 = agent_12.get_state(junction_vehicle, junction_time)
                    if state12 is not None:
                        agent_12.take_action(state12, junction_vehicle)
                        previous_node = agent_12.update_onestep()
                        if previous_node == 1:
                            cost1 += agent_1.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)
                        agent_12.coordinate(junction_fuel, junction_link_time)

                elif junction_id == 13:
                    state13 = agent_13.get_state(junction_vehicle, junction_time)
                    if state13 is not None:
                        agent_13.take_action(state13, junction_vehicle)
                        previous_node = agent_13.update_onestep()
                        if previous_node == 9:
                            cost9 += agent_9.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)
                        agent_13.coordinate(junction_fuel, junction_link_time)
                                    

                # the start point id coordination
                elif junction_id == 1:
                    state1 = agent_1.get_state(junction_vehicle, junction_time)
                    if state1 is not None:
                        agent_1.take_action(state1, junction_vehicle)
                        previous_node = agent_1.update_onestep()
                        agent_1.coordinate(junction_fuel, junction_link_time)

                elif junction_id == 4:
                    state4 = agent_4.get_state(junction_vehicle, junction_time)
                    if state4 is not None:
                        agent_4.take_action(state4, junction_vehicle)
                        previous_node = agent_4.update_onestep()
                        agent_4.coordinate(junction_fuel, junction_link_time)

                # the end point cost calculation
                elif junction_id == 2:
                    state2 = agent_2.get_state(junction_vehicle, junction_time)
                    if state2 is not None:
                        previous_node = agent_2.update_onestep(junction_vehicle)
                        if previous_node == 8:
                            #print('Agent 8 vehicle cost: ', agent_8.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time))
                            cost8 += agent_8.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)
                        elif previous_node == 11:
                            cost11 += agent_11.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)

                elif junction_id == 3:
                    state3 = agent_3.get_state(junction_vehicle, junction_time)
                    if state3 is not None:
                        previous_node = agent_3.update_onestep(junction_vehicle)
                        if previous_node == 11:
                            cost11 += agent_11.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)
                        elif previous_node == 13:
                            cost13 += agent_13.add_to_memory(junction_vehicle, junction_fuel, junction_link_time)


    #agent_1.save_model()
    #agent_4.save_model()
    #agent_5.save_model()
    #agent_6.save_model()
    #agent_7.save_model()
    #agent_8.save_model()
    #agent_9.save_model()
    #agent_10.save_model()
    #agent_11.save_model()
    #agent_12.save_model()
    #agent_13.save_model()

    #np.save('q1', q1_list)
    #np.save('q4', q4_list)
    #np.save('q5', q5_list)
    #np.save('q6', q6_list)
    #np.save('q7', q7_list)
    #np.save('q8', q8_list)
    #np.save('q9', q9_list)
    #np.save('q10', q10_list)
    #np.save('q11', q11_list)
    #np.save('q12', q12_list)
    #np.save('q13', q13_list)


    total_cost = cost8 + cost13 + cost11 + cost7 + cost10 + cost6 + cost12 + cost9 + cost5 + cost1 + cost4

    '''
    print('cost1: ', cost1)
    print('cost4: ', cost4)
    print('cost5: ', cost5)
    print('cost6: ', cost6)
    print('cost7: ', cost7)
    print('cost8: ', cost8)
    print('cost9: ', cost9)
    print('cost10: ', cost10)
    print('cost11: ', cost11)
    print('cost12: ', cost12)
    print('cost13: ', cost13)
    '''
    
    
    print('Episode: ', episode)
    #print('total cost: ', total_cost)
    # save the final results as np file
    np.save('Episode_' + str(episode), total_cost)
    # record total cost for plot
    total_cost_list.append(total_cost)

    print('total cost: ', env.return_cost_cav())

#plt.plot(episode_list, total_cost_list)
#plt.show()





        




