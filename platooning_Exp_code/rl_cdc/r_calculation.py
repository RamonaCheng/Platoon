'''
@Author: WANG Maonan
@Date: 2023-04-13 23:31:38
@Description: 
@LastEditTime: 2023-04-19 20:52:00
'''
import numpy as np
import random
import matplotlib.pyplot as plt
from scipy import integrate
from scipy.optimize import fsolve
import parameters_input as para
LEARNING_RATE, MAX_MEMORY, BATCH_SIZE, GAMMA, gamma, w_1, w_2, d1, collision_time_delay, exploration_rate = para.nominal_values()


# initilizaion
#alpha = 6.78 * 10 ** (-7)
alpha = 3.51 * 10 ** (-7)
#v = 28.0
v = 24.0
eta = 0.1
#k = 41.0 / 100000.0
k = 32.2 / 100000.0
#w_1 = 25.8 / 3600 # value of time
w_2 = 0.868 	  # oil price
#gamma = 0.9



def r_calculaton(arrival_rate, w_1, d2):
	temp = 4.0 * d2 * w_2 * eta * k * arrival_rate / (2.0 * alpha * w_2 * v**3 - w_1)

	temp_flag = 2.0*alpha*w_2*v**3 - w_1

	if temp_flag >= 0:
		r = (1.0/arrival_rate) * np.log(0.5 + 0.5 * (temp + 1.0)**0.5) * 0.5
	else:
		r = 70.0

	#print('temp_flag: ', 2.0*alpha*w_2*v**3 - w_1)
	return r