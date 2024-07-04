'''
@Author: WANG Maonan
@Date: 2023-03-31 10:29:09
@Description: 利用每个节点时间的估计
@LastEditTime: 2023-04-14 15:17:20
'''
import argparse
import numpy as np

import traci
import simpla
import os
import sys
import json
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
parser.add_argument('--lambda_travel', type=float, default=1)
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
Qmatrix_file = open('./Qmatirx_table.json', 'w', encoding='utf-8') # 存储 Q 值
lane_speed_all_file = open('./lane_speed_all.json', 'w', encoding='utf-8') # 存储 link 的 avg speed
lane_speed_filter_file = open('./lane_speed_filter.json', 'w', encoding='utf-8') # 存储 link 的 avg speed（去除探测器后面的车）

print("---------------------------------")
print("Simulation starts")
print("---------------------------------")

episode_list = range(1)
total_cost_list = []


def calculate_avg_speed():
    lane_speed_all = {
        "link1_0": list(),
        "link2_0": list(),
        "link3_0": list(),
        "link4_0": list(),
        "link5_0": list(),
        "link6_0": list(),
        "link7_0": list(),
        "link8_0": list(),
        "link9_0": list(),
        "link10_0": list(),
        "link11_0": list(),
        "link12_0": list(),
        "link13_0": list(),
        "link14_0": list(),
        "link15_0": list(),
        "link16_0": list(),
        "link17_0": list(),
        "link18_0": list(),
        "link19_0": list(),
        "start1_0": list(),
        "start2_0": list(),
        "end1_0": list(),
        "end2_0": list(),
    }
    lane_speed_filter = {
        "link1_0": list(),
        "link2_0": list(),
        "link3_0": list(),
        "link4_0": list(),
        "link5_0": list(),
        "link6_0": list(),
        "link7_0": list(),
        "link8_0": list(),
        "link9_0": list(),
        "link10_0": list(),
        "link11_0": list(),
        "link12_0": list(),
        "link13_0": list(),
        "link14_0": list(),
        "link15_0": list(),
        "link16_0": list(),
        "link17_0": list(),
        "link18_0": list(),
        "link19_0": list(),
        "start1_0": list(),
        "start2_0": list(),
        "end1_0": list(),
        "end2_0": list(),
    }
    for veh_id in traci.vehicle.getIDList():
        lane_id = traci.vehicle.getLaneID(veh_id) # lane
        pos = traci.vehicle.getLanePosition(veh_id) # position
        veh_speed = traci.vehicle.getSpeed(veh_id) # speed, 要是没有车, 速度就是 24
        lane_length = traci.lane.getLength(lane_id) # lane 的长度
        if lane_id in lane_speed_all:
            lane_speed_all[lane_id].append(veh_speed)
            if pos > 50 and pos < lane_length-600:
                lane_speed_filter[lane_id].append(veh_speed)
    
    return lane_speed_filter, lane_speed_all

# begin the episode
for episode in range(1):
    learning_rate = 1.0
    # reset the environment
    arrival_junction, arrival_vehicle, arrival_time, arrival_fuel, arrival_link_time = env.reset()
    agent_1.reset(learning_rate, args.lambda_travel)
    agent_2.reset(learning_rate, args.lambda_travel)
    agent_3.reset(learning_rate, args.lambda_travel)
    agent_4.reset(learning_rate, args.lambda_travel)
    agent_5.reset(learning_rate, args.lambda_travel)
    agent_6.reset(learning_rate, args.lambda_travel)
    agent_7.reset(learning_rate, args.lambda_travel)
    agent_8.reset(learning_rate, args.lambda_travel)
    agent_9.reset(learning_rate, args.lambda_travel)
    agent_10.reset(learning_rate, args.lambda_travel)
    agent_11.reset(learning_rate, args.lambda_travel)
    agent_12.reset(learning_rate, args.lambda_travel)
    agent_13.reset(learning_rate, args.lambda_travel)
    # reset the time table
    Qmatrix_1 = agent_1.return_time_matrix()
    Qmatrix_4 = agent_4.return_time_matrix()
    Qmatrix_5 = agent_5.return_time_matrix()
    Qmatrix_6 = agent_6.return_time_matrix()
    Qmatrix_7 = agent_7.return_time_matrix()
    Qmatrix_8 = agent_8.return_time_matrix()
    Qmatrix_9 = agent_9.return_time_matrix()
    Qmatrix_10 = agent_10.return_time_matrix()
    Qmatrix_11 = agent_11.return_time_matrix()
    Qmatrix_12 = agent_12.return_time_matrix()
    Qmatrix_13 = agent_13.return_time_matrix()
    Qmatrix_table = [Qmatrix_1, Qmatrix_4, Qmatrix_5, Qmatrix_6, Qmatrix_7, Qmatrix_8, Qmatrix_9, Qmatrix_10, Qmatrix_11, Qmatrix_12, Qmatrix_13]
    # initialize total cost
    total_cost, cost1, cost4, cost5, cost6, cost7, cost8, cost9, cost10, cost11, cost12, cost13 = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    # begin the simulation
    while (traci.simulation.getMinExpectedNumber()>0) and (traci.simulation.getTime() < 4001):
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


                if junction_id == 5:
                    state5 = agent_5.get_state(junction_vehicle, junction_time)
                    if state5 is not None:
                        agent_5.take_action(state5, Qmatrix_table)
                        previous_node, min_q = agent_5.update_onestep()
                        #q5_list.append([i, min_q])
                        #print('Agent 5 Min Q: ', min_q)
                        if previous_node == 1:
                            #print('Agent 5 Min Q: ', min_q)
                            cost1 += agent_1.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_1.experience_replay()
                            Qmatrix_table[0] = agent_1.return_time_matrix()
                        elif previous_node == 4:
                            #print('min q from junction 5: ', min_q)
                            cost4 += agent_4.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_4.experience_replay()
                            Qmatrix_table[1] = agent_4.return_time_matrix()
                        agent_5.coordinate(junction_fuel, junction_link_time)
                
                elif junction_id == 6:
                    state6 = agent_6.get_state(junction_vehicle, junction_time)
                    if state6 is not None:
                        agent_6.take_action(state6, Qmatrix_table) # 只计算动作
                        previous_node, min_q = agent_6.update_onestep() # 估计 next state 的值
                        #q6_list.append([i, min_q])
                        #print('Agent 6 Min Q: ', min_q)
                        if previous_node == 5:
                            cost5 += agent_5.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_5.experience_replay()
                            Qmatrix_table[2] = agent_5.return_time_matrix()
                        elif previous_node == 12:
                            cost12 += agent_12.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_12.experience_replay()
                            Qmatrix_table[9] = agent_12.return_time_matrix()
                        agent_6.coordinate(junction_fuel, junction_link_time)

                elif junction_id == 7:
                    state7 = agent_7.get_state(junction_vehicle, junction_time)
                    if state7 is not None:
                        agent_7.take_action(state7, Qmatrix_table)
                        previous_node, min_q = agent_7.update_onestep()
                        #q7_list.append([i, min_q])
                        #print('Agent 7 Min Q: ', min_q)
                        if previous_node == 6:
                            cost6 += agent_6.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_6.experience_replay()
                            Qmatrix_table[3] = agent_6.return_time_matrix()
                        agent_7.coordinate(junction_fuel, junction_link_time)

                elif junction_id == 8:
                    state8 = agent_8.get_state(junction_vehicle, junction_time)
                    #print('State 8: ', state8)
                    if state8 is not None:
                        agent_8.take_action(state8, Qmatrix_table)
                        previous_node, min_q = agent_8.update_onestep()
                        #q8_list.append([i, min_q])
                        #print('Agent 8 Min Q: ', min_q)
                        if previous_node == 7:
                            cost7 += agent_7.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_7.experience_replay()
                            Qmatrix_table[4] = agent_7.return_time_matrix()
                        elif previous_node == 12:
                            cost12 += agent_12.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_12.experience_replay()
                            Qmatrix_table[9] = agent_12.return_time_matrix()
                        agent_8.coordinate(junction_fuel, junction_link_time)

                elif junction_id == 9:
                    state9 = agent_9.get_state(junction_vehicle, junction_time)
                    if state9 is not None:
                        agent_9.take_action(state9, Qmatrix_table)
                        previous_node, min_q = agent_9.update_onestep()
                        #q9_list.append([i, min_q])
                        #print('Agent 9 Min Q: ', min_q)
                        if previous_node == 4:
                            #print('min q from junction 9: ', min_q)
                            cost4 += agent_4.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_4.experience_replay()
                            Qmatrix_table[1] = agent_4.return_time_matrix()
                        elif previous_node == 5:
                            cost5 += agent_5.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_5.experience_replay()
                            Qmatrix_table[2] = agent_5.return_time_matrix()
                        agent_9.coordinate(junction_fuel, junction_link_time)

                elif junction_id == 10:
                    state10 = agent_10.get_state(junction_vehicle, junction_time)
                    if state10 is not None:
                        agent_10.take_action(state10, Qmatrix_table)
                        previous_node, min_q = agent_10.update_onestep()
                        #q10_list.append([i, min_q])
                        #print('Agent 10 Min Q: ', min_q)
                        if previous_node == 6:
                            cost6 += agent_6.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_6.experience_replay()
                            Qmatrix_table[3] = agent_6.return_time_matrix()
                        elif previous_node == 9:
                            cost9 += agent_9.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_9.experience_replay()
                            Qmatrix_table[6] = agent_9.return_time_matrix()
                        agent_10.coordinate(junction_fuel, junction_link_time)

                elif junction_id == 11:
                    state11 = agent_11.get_state(junction_vehicle, junction_time)
                    if state11 is not None:
                        agent_11.take_action(state11, Qmatrix_table)
                        previous_node, min_q = agent_11.update_onestep()
                        #q11_list.append([i, min_q])
                        #print('Agent 11 Min Q: ', min_q)
                        if previous_node == 7:
                            cost7 += agent_7.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_7.experience_replay()
                            Qmatrix_table[4] = agent_7.return_time_matrix()
                        elif previous_node == 10:
                            cost10 += agent_10.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_10.experience_replay()
                            Qmatrix_table[7] = agent_10.return_time_matrix()
                        agent_11.coordinate(junction_fuel, junction_link_time)

                elif junction_id == 12:
                    state12 = agent_12.get_state(junction_vehicle, junction_time)
                    if state12 is not None:
                        agent_12.take_action(state12, Qmatrix_table)
                        previous_node, min_q = agent_12.update_onestep()
                        #q12_list.append([i, min_q])
                        #print('Agent 12 Min Q: ', min_q)
                        if previous_node == 1:
                            #print('Agent 12 Min Q: ', min_q)
                            cost1 += agent_1.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_1.experience_replay()
                            Qmatrix_table[0] = agent_1.return_time_matrix()
                        agent_12.coordinate(junction_fuel, junction_link_time)

                elif junction_id == 13:
                    state13 = agent_13.get_state(junction_vehicle, junction_time)
                    if state13 is not None:
                        agent_13.take_action(state13, Qmatrix_table)
                        previous_node, min_q = agent_13.update_onestep()
                        #q13_list.append([i, min_q])
                        #print('Agent 13 Min Q: ', min_q)
                        if previous_node == 9:
                            cost9 += agent_9.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_9.experience_replay()
                            Qmatrix_table[6] = agent_9.return_time_matrix()
                        agent_13.coordinate(junction_fuel, junction_link_time)
                                    

                # the start point id coordination
                elif junction_id == 1:
                    state1 = agent_1.get_state(junction_vehicle, junction_time)
                    if state1 is not None:
                        agent_1.take_action(state1, Qmatrix_table)
                        previous_node, min_q = agent_1.update_onestep()
                        #print('Agent 1 Min Q: ', min_q)
                        #q1_list.append([i, min_q])
                        agent_1.coordinate(junction_fuel, junction_link_time)

                elif junction_id == 4:
                    state4 = agent_4.get_state(junction_vehicle, junction_time)
                    if state4 is not None:
                        agent_4.take_action(state4, Qmatrix_table)
                        previous_node, min_q = agent_4.update_onestep()
                        #q4_list.append([i, min_q])
                        agent_4.coordinate(junction_fuel, junction_link_time)

                # the end point cost calculation
                elif junction_id == 2:
                    state2 = agent_2.get_state(junction_vehicle, junction_time)
                    if state2 is not None:
                        previous_node, min_q = agent_2.update_onestep(junction_vehicle)
                        #print('Agent 2 Min Q: ', min_q)
                        if previous_node == 8:
                            #print('Agent 8 vehicle cost: ', agent_8.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time))
                            cost8 += agent_8.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_8.experience_replay()
                            Qmatrix_table[5] = agent_8.return_time_matrix()
                        elif previous_node == 11:
                            cost11 += agent_11.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_11.experience_replay()
                            Qmatrix_table[8] = agent_11.return_time_matrix()

                elif junction_id == 3:
                    state3 = agent_3.get_state(junction_vehicle, junction_time)
                    if state3 is not None:
                        previous_node, min_q = agent_3.update_onestep(junction_vehicle)
                        #print('Agent 3 Min Q: ', min_q)
                        if previous_node == 11:
                            cost11 += agent_11.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_11.experience_replay()
                            Qmatrix_table[8] = agent_11.return_time_matrix()
                        elif previous_node == 13:
                            cost13 += agent_13.add_to_memory(junction_vehicle, min_q, junction_fuel, junction_link_time)
                            agent_13.experience_replay()
                            Qmatrix_table[10] = agent_13.return_time_matrix()
                
        # json.dump({traci.simulation.getTime():Qmatrix_table}, Qmatrix_file, indent=4, ensure_ascii=False)
        if traci.simulation.getTime() % 1 == 0:
            # 存储 Q 值
            json.dump({'time': traci.simulation.getTime(), 'Q': Qmatrix_table}, Qmatrix_file, ensure_ascii=False)
            Qmatrix_file.write('\n')
            # 存储 avg speed
            lane_speed_filter, lane_speed_all = calculate_avg_speed()
            json.dump(
                {
                    'time': traci.simulation.getTime(), 
                    'speed': {lane_id:np.mean(lane_speed_filter[lane_id]) if len(lane_speed_filter[lane_id])>0 else 24 for lane_id in lane_speed_filter},
                },
                lane_speed_filter_file, ensure_ascii=False
            )
            lane_speed_filter_file.write('\n')
            json.dump(
                {
                    'time': traci.simulation.getTime(), 
                    'speed': {lane_id:np.mean(lane_speed_all[lane_id]) if len(lane_speed_all[lane_id])>0 else 24 for lane_id in lane_speed_all},
                },
                lane_speed_all_file, ensure_ascii=False
            )
            lane_speed_all_file.write('\n')
    
    # 关闭 agent 的 csv 文件
    agent_1.close_csv()
    agent_4.close_csv()
    agent_5.close_csv()
    agent_6.close_csv()
    agent_7.close_csv()
    agent_8.close_csv()
    agent_9.close_csv()
    agent_10.close_csv()
    agent_11.close_csv()
    agent_12.close_csv()
    agent_13.close_csv()
    Qmatrix_file.close()
    lane_speed_all_file.close()
    lane_speed_filter_file.close()

    total_cost = cost8 + cost13 + cost11 + cost7 + cost10 + cost6 + cost12 + cost9 + cost5 + cost1 + cost4

    print('Episode: ', episode)
    print('Learning rate: ', learning_rate)
    #print('total cost: ', total_cost)
    # save the final results as np file
    np.save('Episode_' + str(episode), total_cost)
    # record total cost for plot
    #total_cost_list.append(total_cost)

    print('total cost: ', env.return_cost_cav()) # 使用这个式子

#plt.plot(episode_list, total_cost_list)
#plt.show()





        




