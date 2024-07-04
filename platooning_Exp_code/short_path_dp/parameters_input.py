'''
@Author: WANG Maonan
@Date: 2023-04-13 23:31:38
@Description: 
@LastEditTime: 2023-04-14 00:12:40
'''
def nominal_values():

    #LEARNING_RATE = 1e-3
    LEARNING_RATE = 0.0
    MAX_MEMORY = 1000000
    BATCH_SIZE = 1
    GAMMA = 1.0       # use to calculate the next Q value
    gamma = 0.9       # use to calculate the threshold and C value 


    # platooning parameters
    #w_1 = 25.8 / 3600   # value of time ($/s)
    #w_2 = 0.868         # oil price ($/L)
    w_1 = 25.8           # value of time ($/hour)
    w_2 = 0.868          # oil price ($/L)
    d1 = 500.0          # the distance of d_1 (m)
    collision_time_delay = 0.7

    return LEARNING_RATE, MAX_MEMORY, BATCH_SIZE, GAMMA, gamma, w_1, w_2, d1, collision_time_delay
