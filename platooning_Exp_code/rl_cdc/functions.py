'''
@Author: WANG Maonan
@Date: 2023-04-13 20:58:56
@Description: 
@LastEditTime: 2023-04-14 21:36:22
'''
import numpy as np
import parameters_input as para
from dijkstar import Graph, find_path


# read nominal parameters
LEARNING_RATE, MAX_MEMORY, BATCH_SIZE, GAMMA, gamma, w_1, w_2, d1, collision_time_delay, exploration_rate = para.nominal_values()


link_length_list = np.array([
	2.0, 2.0, 2.0, 2.8, 2.0, 
	2.0, 2.0, 2.0, 2.0, 2.0, 
	2.0, 2.0, 2.8, 2.0, 2.0, 
	2.0, 2.0, 4.5, 2.0]
) * 1000.0

nominal_speed = 24.0

node_info = {
	1:{5: 1, 12: 2},
	2:{},
	3:{},
	4:{5: 3, 9: 4},
	5:{6: 5, 9: 6},
	6:{7: 7, 10: 8},
	7:{8: 9, 11: 10},
	8:{2: 11},
	9:{10: 12, 13: 13},
	10:{11: 14},
	11:{2: 15, 3: 16},
	12:{6: 17, 8: 18},
	13:{3: 19},
} # 保存 node:{next_node:next_link, ...}


def fuel_rate(vehicle_speed):
	return (3.51 * (10 ** (-4)) * (vehicle_speed ** 3) + 0.407 * vehicle_speed) 	# ml/s


def Dijkstar(origin, destination):
	graph = Graph()
	graph.add_edge(1, 5, link_length_list[0])       	#link 1
	graph.add_edge(1, 12, link_length_list[1])      	#link 2
	graph.add_edge(4, 5, link_length_list[2])      		#link 3
	graph.add_edge(4, 9, link_length_list[3])      		#link 4
	graph.add_edge(5, 6, link_length_list[4])      		#link 5
	graph.add_edge(5, 9, link_length_list[5])      		#link 6
	graph.add_edge(6, 7, link_length_list[6])      		#link 7
	graph.add_edge(6, 10, link_length_list[7])      	#link 8
	graph.add_edge(7, 8, link_length_list[8])      		#link 9
	graph.add_edge(7, 11, link_length_list[9])      	#link 10
	graph.add_edge(8, 2, link_length_list[10])      	#link 11
	graph.add_edge(9, 10, link_length_list[11])      	#link 12
	graph.add_edge(9, 13, link_length_list[12])      	#link 13
	graph.add_edge(10, 11, link_length_list[13])      	#link 14
	graph.add_edge(11, 2, link_length_list[14])      	#link 15
	graph.add_edge(11, 3, link_length_list[15])      	#link 16
	graph.add_edge(12, 6, link_length_list[16])      	#link 17
	graph.add_edge(12, 8, link_length_list[17])      	#link 18
	graph.add_edge(13, 3, link_length_list[18])      	#link 19

	#return find_path(graph, origin, destination).total_cost
	return find_path(graph, origin, destination)



def return_cost_not_catch_up(current_node, next_node, destination_node):
	vehicle_speed = nominal_speed
	next_link_id = node_info[current_node][next_node]
	
	travel_distance = (d1 + link_length_list[next_link_id-1] + Dijkstar(next_node, destination_node).total_cost) * 1.0 # m
	
	travel_time = travel_distance / vehicle_speed # s

	return travel_time

for _node_id, next_info in node_info.items():
	_node_travel = list()
	for dest in [2, 3]: # 终点
		_travel_time_list = list()
		for _next_id, _ in next_info.items():
			try:
				travel_time = return_cost_not_catch_up(_node_id, _next_id, dest)
				_travel_time_list.append(travel_time)
			except:
				pass
		if len(_travel_time_list) > 0:
			_node_travel.append(_travel_time_list)
	print(f'{_node_id}, {_node_travel}')