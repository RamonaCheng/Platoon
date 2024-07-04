import numpy as np
import random
import matplotlib.pyplot as plt

# initilizaion
#alpha = 7.84 * 10 ** (-7)
alpha = 3.51 * 10 ** (-7)
d1 = 1000.0
v = 28.0
#v = 23.0
eta = 0.1
#k = 41.0 / 100000.0
k = 32.2 / 100000.0
d2 = 30000.0
#d2 = 100000.0
w_1 = 25.8 / 3600 # value of time
#w_2 = 0.868 	  # oil price
w_2 = 0.0 	  # oil price
gamma = 0.9


def G(sk):
	return w_1 * sk + w_2 * (alpha * d1 * (v ** 2) - alpha * d1 * ((d1 / (d1 / v - sk)) ** 2) + eta * k * d2)

def G_derivative(sk):
	return w_1 - 2 * w_2 * alpha * ((d1 / (d1 / v - sk)) ** 3)

s_list = np.linspace(-100.0, (d1/v-10.0), num=100)

gs_list = []

for sk in s_list:

	gs_list.append(G(sk))

plt.plot(s_list, gs_list)
plt.show()