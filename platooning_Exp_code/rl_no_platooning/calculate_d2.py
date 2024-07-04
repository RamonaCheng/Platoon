'''
@Author: WANG Maonan
@Date: 2023-04-13 23:31:38
@Description: 
@LastEditTime: 2023-04-14 21:43:41
'''
import parameters_input as para
LEARNING_RATE, MAX_MEMORY, BATCH_SIZE, GAMMA, gamma, w_1, w_2, d1, collision_time_delay, exploration_rate = para.nominal_values()


alpha = 3.51 * 10 ** (-7)
beta = 4.07 * 10 ** (-4)
k = 32.2 / 100000.0

agent_list = [1, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]


d2_link1 = 2.0 * 1000.0     # link 1 distance (m)
d2_link2 = 2.0 * 1000.0     # link 2 distance (m)
d2_link3 = 2.0 * 1000.0     # link 3 distance (m)
d2_link4 = 2.8 * 1000.0    # link 4 distance (m)
d2_link5 = 2.0 * 1000.0     # link 5 distance (m)
d2_link6 = 2.0 * 1000.0     # link 6 distance (m)
d2_link7 = 2.0 * 1000.0     # link 7 distance (m)
d2_link8 = 2.0 * 1000.0    # link 8 distance (m)
d2_link9 = 2.0 * 1000.0     # link 9 distance (m)
d2_link10 = 2.0 * 1000.0    # link 10 distance (m)
d2_link11 = 2.0 * 1000.0    # link 11 distance (m)
d2_link12 = 2.0 * 1000.0   # link 12 distance (m)
d2_link13 = 2.8 * 1000.0    # link 13 distance (m)
d2_link14 = 2.0 * 1000.0    # link 14 distance (m)
d2_link15 = 2.0 * 1000.0    # link 15 distance (m)
d2_link16 = 2.0 * 1000.0    # link 16 distance (m)
d2_link17 = 2.0 * 1000.0    # link 17 distance (m)
d2_link18 = 4.5 * 1000.0   # link 18 distance (m)
d2_link19 = 2.0 * 1000.0   # link 19 distance (m)

def decide_agent(current_agent, temp_des, Qmatrix):
    if current_agent == 1:
        next_node, travel_time, link_distance = agent1_link(temp_des, Qmatrix)
    elif current_agent == 4:
        next_node, travel_time, link_distance = agent4_link(temp_des, Qmatrix)
    elif current_agent == 5:
        next_node, travel_time, link_distance = agent5_link(temp_des, Qmatrix)
    elif current_agent == 6:
        next_node, travel_time, link_distance = agent6_link(temp_des, Qmatrix)
    elif current_agent == 7:
        next_node, travel_time, link_distance = agent7_link(temp_des, Qmatrix)
    elif current_agent == 8:
        next_node, travel_time, link_distance = agent8_link(temp_des, Qmatrix)
    elif current_agent == 9:
        next_node, travel_time, link_distance = agent9_link(temp_des, Qmatrix)
    elif current_agent == 10:
        next_node, travel_time, link_distance = agent10_link(temp_des, Qmatrix)
    elif current_agent == 11:
        next_node, travel_time, link_distance = agent11_link(temp_des, Qmatrix)
    elif current_agent == 12:
        next_node, travel_time, link_distance = agent12_link(temp_des, Qmatrix)
    elif current_agent == 13:
        next_node, travel_time, link_distance = agent13_link(temp_des, Qmatrix)

    return next_node, travel_time, link_distance


def agent1_link(temp_des, Qmatrix):
    if temp_des == 2:
        q_values = Qmatrix[0]
        if q_values[0] <= q_values[1]:
            action = 1
        else:
            action = 2
        travel_time = min(q_values)
    elif temp_des == 3:
        q_values = Qmatrix[1]
        if q_values[0] <= q_values[1]:
            action = 1
        else:
            action = 2
        travel_time = min(q_values)

    if action == 1:
        next_node = 5
        link_distance = d2_link1
    elif action == 2:
        next_node = 12
        link_distance = d2_link2

    return next_node, travel_time, link_distance


def agent4_link(temp_des, Qmatrix):
    if temp_des == 2:
        q_values = Qmatrix[0]
        if q_values[0] <= q_values[1]:
            action = 3
        else:
            action = 4
        travel_time = min(q_values)
    elif temp_des == 3:
        q_values = Qmatrix[1]
        if q_values[0] <= q_values[1]:
            action = 3
        else:
            action = 4
        travel_time = min(q_values)

    if action == 3:
        next_node = 5
        link_distance = d2_link3
    elif action == 4:
        next_node = 9
        link_distance = d2_link4

    return next_node, travel_time, link_distance


def agent5_link(temp_des, Qmatrix):
    if temp_des == 2:
        q_values = Qmatrix[0]
        if q_values[0] <= q_values[1]:
            action = 5
        else:
            action = 6
        travel_time = min(q_values)
    elif temp_des == 3:
        q_values = Qmatrix[1]
        if q_values[0] <= q_values[1]:
            action = 5
        else:
            action = 6
        travel_time = min(q_values)

    if action == 5:
        next_node = 6
        link_distance = d2_link5
    elif action == 6:
        next_node = 9
        link_distance = d2_link6
    
    return next_node, travel_time, link_distance


def agent6_link(temp_des, Qmatrix):
    if temp_des == 2:
        q_values = Qmatrix[0]
        if q_values[0] <= q_values[1]:
            action = 7
        else:
            action = 8
        travel_time = min(q_values)
    elif temp_des == 3:
        q_values = Qmatrix[1]
        if q_values[0] <= q_values[1]:
            action = 7
        else:
            action = 8
        travel_time = min(q_values)

    if action == 7:
        next_node = 7
        link_distance = d2_link7
    elif action == 8:
        next_node = 10
        link_distance = d2_link8

    return next_node, travel_time, link_distance


def agent7_link(temp_des, Qmatrix):
    if temp_des == 2:
        q_values = Qmatrix[0]
        if q_values[0] <= q_values[1]:
            action = 9
        else:
            action = 10
        travel_time = min(q_values)
    elif temp_des == 3:
        action = 10
        travel_time = Qmatrix[1][0]

    if action == 9:
        next_node = 8
        link_distance = d2_link9
    elif action == 10:
        next_node = 11
        link_distance = d2_link10
    
    return next_node, travel_time, link_distance


def agent8_link(temp_des, Qmatrix):
    action = 11

    next_node = 2

    link_distance = d2_link11

    travel_time = Qmatrix[0]
            
    return next_node, travel_time, link_distance


def agent9_link(temp_des, Qmatrix):
    if temp_des == 3:
        q_values = Qmatrix[1]
        if q_values[0] <= q_values[1]:
            action = 12
        else:
            action = 13
        travel_time = min(q_values)
    elif temp_des == 2:
        action = 12
        travel_time = Qmatrix[0][0]

    if action == 12:
        next_node = 10
        link_distance = d2_link12
    elif action == 13:
        next_node = 13
        link_distance = d2_link13
    
    return next_node, travel_time, link_distance


def agent10_link(temp_des, Qmatrix):
    action = 14

    next_node = 11

    link_distance = d2_link14

    if temp_des == 2:
        travel_time = Qmatrix[0]
    elif temp_des == 3:
        travel_time = Qmatrix[1]
            
    return next_node, travel_time, link_distance


def agent11_link(temp_des, Qmatrix):
    if temp_des == 2:
        action = 15
        travel_time = Qmatrix[0]
    elif temp_des == 3:
        action = 16
        travel_time = Qmatrix[1]

    if action == 15:
        next_node = 2
        link_distance = d2_link15
    elif action == 16:
        next_node = 3
        link_distance = d2_link16
            
    return next_node, travel_time, link_distance


def agent12_link(temp_des, Qmatrix):
    if temp_des == 2:
        q_values = Qmatrix[0]
        if q_values[0] <= q_values[1]:
            action = 17
        else:
            action = 18
        travel_time = min(q_values)
    elif temp_des == 3:
        action = 17
        travel_time = Qmatrix[1][0]

    if action == 17:
        next_node = 6
        link_distance = d2_link17
    elif action == 18:
        next_node = 8
        link_distance = d2_link18
            
    return next_node, travel_time, link_distance


def agent13_link(temp_des, Qmatrix):
    action = 19

    next_node = 3

    link_distance = d2_link19

    travel_time = Qmatrix[0]
            
    return next_node, travel_time, link_distance



def return_path(current_node, destination_node, Qmatrix_table):
    node_list = []

    while current_node != destination_node:
        agent_num = agent_list.index(current_node)
        next_node, travel_time, link_distance = decide_agent(current_node, destination_node, Qmatrix_table[agent_num])

        current_node = next_node
        node_list.append(current_node)

    return node_list


def return_distance(current_node, destination_node, Qmatrix_table):
    path_distance = 0

    while current_node != destination_node:
        agent_num = agent_list.index(current_node)
        next_node, travel_time, link_distance = decide_agent(current_node, destination_node, Qmatrix_table[agent_num])

        current_node = next_node
        path_distance += link_distance

    return path_distance


def return_initial_Qmatrix_Table():
    Qmatrix_table = [
                [[437.5, 375.0], [387.5, 437.5]], 
                [[437.5, 387.5], [387.5, 337.5]], 
                [[354.1666666666667, 354.1666666666667], [354.1666666666667, 304.1666666666667]],
                [[270.8333333333333, 270.8333333333333], [270.8333333333333, 270.8333333333333]], 
                [[187.5, 187.5], [187.5, 187.5]], 
                [104.16666666666667, 104.16666666666667], 
                [[270.8333333333333, 270.8333333333333], [270.8333333333333, 220.83333333333334]],
                [187.5, 187.5], 
                [104.16666666666667, 104.16666666666667], 
                [[354.1666666666667, 291.6666666666667], [354.1666666666667, 354.1666666666667]], 
                [104.16666666666667]
            ]

    return Qmatrix_table


def calculate_equivalent_distance(travel_time, travel_distance):
    travel_speed = travel_distance / travel_time
    fuel_consumption = alpha * travel_distance * travel_speed * travel_speed + beta * travel_distance

    distance = fuel_consumption / k

    return distance


def d2_distance(current_node, lead_des, temp_des, lead_link, temp_link, Qmatrix_table, lead_path, temp_path):
    d2_distance = 0

    if temp_link != lead_link:
        d2_distance = 0
    else:
        # calculate the original distance
        agent_num = agent_list.index(current_node)
        next_node, travel_time, link_distance = decide_agent(current_node, temp_des, Qmatrix_table[agent_num])
        original_time = travel_time
        original_distance = return_distance(current_node, temp_des, Qmatrix_table)
        original_equiva_distance = calculate_equivalent_distance(original_time, original_distance)


        # calculate the driving alone distance

        # decide the path for the leading vehicle and the current vehicle
        lead_node_list = lead_path
        temp_node_list = temp_path

        #print('des: ', lead_des, temp_des)
        #print('list: ', lead_node_list, temp_node_list)

        if lead_node_list == temp_node_list:
            d2_distance = original_equiva_distance
        else:
            i = 0
            while lead_node_list[i] == temp_node_list[i]:
                i += 1
            # decide the split node
            split_node = temp_node_list[i-1]
            #print('split_node: ', split_node)
            split_agent_num = agent_list.index(split_node)
            __, split_travel_time, split_link_distance = decide_agent(split_node, temp_des, Qmatrix_table[split_agent_num])

            split_equiva_distance = calculate_equivalent_distance(split_travel_time, split_link_distance)

            d2_distance = original_equiva_distance - split_equiva_distance

 
    return d2_distance


#print(d2_distance(1, 2, 3, 1, 1, Qmatrix_table))



