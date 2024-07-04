'''
@Author: WANG Maonan
@Date: 2023-03-31 10:29:09
@Description: 
@LastEditTime: 2023-03-31 21:35:04
'''
import numpy as np
import random
import matplotlib.pyplot as plt
from scipy import integrate
from scipy.optimize import fsolve
import math
#import sympy as sym
#import math
#import sympy

#sym.init_printing()

# initilizaion
#alpha = 7.84 * 10 ** (-7)
alpha = 3.51 * 10 ** (-7)
d1 = 1000.0
#v = 28.0
#v = 18.26
eta = 0.1
#k = 41.0 / 100000.0
k = 32.2 / 100000.0
#d2 = 30000.0
#d2 = 100000.0
#w_1 = 25.8 / 3600 # value of time
#w_2 = 0.868 	  # oil price
#w_2 = 0.3
#gamma = 0.9

#arrival_rate = 0.1

def reward(sk, w_1, d2, v, w_2):
	reward_catch_up = alpha * d1 * v**2 + eta * k * d2 - alpha * d1 * (d1 / (d1 / v - sk))**2
	reward_catch_up = reward_catch_up * w_2 + sk * w_1

	return reward_catch_up

def derivative_reward(sk, w_1, v, w_2):

	derivative_reward_catch_up = -2.0 * w_2 * alpha * d1**3 * (1.0 / ((d1 / v - sk) ** 3)) + w_1
	return derivative_reward_catch_up


def theta_c(arrival_rate, initial_value, w_1, gamma, d2, v, w_2):

	def f1(x):
		return math.exp(- arrival_rate * (1.0 - gamma) * x) * (derivative_reward(x, w_1, v, w_2) - arrival_rate * reward(x, w_1, d2, v, w_2))

	
	# begin to slove the equations

	def f(x):
		Z = float(x[0])
		theta = min(float(x[1]), d1/v-0.5)
		c = float(x[2])

		function_1 = reward(theta, w_1, d2, v, w_2) + gamma * Z - Z

		function_2 = derivative_reward(c, w_1, v, w_2) - arrival_rate * reward(c, w_1, d2, v, w_2) + arrival_rate * (1.0 - gamma) * (Z + reward(0.0, w_1, d2, v, w_2))

		v2, err2 = integrate.quad(f1, c, theta)

		function_3 = np.exp(arrival_rate * (1.0 - gamma) * theta) * (v2 + (Z + reward(0.0, w_1, d2, v, w_2)) * np.exp(- arrival_rate * (1.0 - gamma) * c)) - Z

		return [function_1, function_2, function_3]

	#result = fsolve(f, initial_value, xtol=1.49012e-08)
	result = fsolve(f, initial_value)

	return result