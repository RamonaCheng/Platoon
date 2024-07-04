def nominal_values():

    #LEARNING_RATE = 1e-3
    LEARNING_RATE = 1e-3
    MAX_MEMORY = 1000000
    #MAX_MEMORY = 50000
    #BATCH_SIZE = 20
    BATCH_SIZE = 1
    GAMMA = 1.0
    EXPLORATION_DECAY = 0.995
    EXPLORATION_MIN = 0.01
    #initial_exploration_rate = 1.0
    initial_exploration_rate = 1.0

    # platooning parameters
    #w_1 = 25.8 / 3600   # value of time ($/s)
    #w_2 = 0.868         # oil price ($/L)
    w_1 = 25.8           # value of time ($/hour)
    w_2 = 0.868          # oil price ($/L)
    d1 = 1000.0          # the distance of d_1 (m)
    collision_time_delay = 1.0
    constant_time_reduction = 0.0
    threshold_default_platoon = 0.0

    return LEARNING_RATE, MAX_MEMORY, BATCH_SIZE, GAMMA, EXPLORATION_DECAY, EXPLORATION_MIN, initial_exploration_rate, w_1, w_2, d1, collision_time_delay, constant_time_reduction, threshold_default_platoon
