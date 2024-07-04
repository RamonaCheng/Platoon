import numpy as np
import random
import matplotlib.pyplot as plt
from scipy import integrate
from scipy.optimize import fsolve
import r_calculation as rc
import theta_calculation as tc
import time

# initilizaion
alpha = 3.51 * 10 ** (-7)
d1 = 1000.0
v = 28.0
eta = 0.1
k = 32.2 / 100000.0
d2 = 100000.0
w_1 = 25.8 / 3600 		# value of time
w_2 = 0.868				# oil price
gamma = 0.9
gamma_estimator = 0.9

#arrival_rate = 0.018
#num_veh = 1000 			#1000
#estimated_arrval_rate = 0.012

upper_branch_arrival_rate_list = [0.071, 0.069, 0.058, 0.057, 0.075, 0.110, 0.193, 0.339, 0.380, 0.317, 0.289, 0.263, 0.279, 0.298, 0.335, 0.348, 0.382, 0.372, 0.375, 0.319, 0.235, 0.207, 0.162, 0.115]
lower_branch_arrival_rate_list = [0.185, 0.146, 0.124, 0.113, 0.191, 0.388, 0.879, 1.527, 1.594, 1.367, 1.191, 1.109, 1.170, 1.209, 1.459, 1.547, 1.586, 1.606, 1.577, 1.386, 1.027, 0.815, 0.620, 0.378]

CAV_ratio = 0.01
num_veh = 110000


depart_time_1 = 0.0
depart_time_list_1 = []

depart_time_2 = 5.0
depart_time_list_2 = []

for i_1 in range(num_veh):
	if depart_time_1 < 3600 * 24:
		depart_time_1_index = int(depart_time_1 / 3600.0)

		arrival_rate_1 = upper_branch_arrival_rate_list[depart_time_1_index] * CAV_ratio			

		depart_time_list_1.append(depart_time_1)
		random_num_1 = np.random.random()
		delta_time_1 = -1.0 * np.log(random_num_1) / arrival_rate_1
		depart_time_1 += delta_time_1

for i_2 in range(num_veh):
	if depart_time_2 < 3600 * 24:
		depart_time_2_index = int(depart_time_2 / 3600.0)
				
		arrival_rate_2 = lower_branch_arrival_rate_list[depart_time_2_index] * CAV_ratio

		depart_time_list_2.append(depart_time_2)
		random_num_2 = np.random.random()
		delta_time_2 = -1.0 * np.log(random_num_2) / arrival_rate_2
		depart_time_2 += delta_time_2

depart_time_list = depart_time_list_1 + depart_time_list_2
depart_time_list.sort()



gamma_list = range(1, 10)
w_1_list = range(5, 41, 1)
arrival_rate_list = range(1,100)

d2_list = range(50000, 300001, 50000)


#runner_mode = 'CDC'
runner_mode_list = ['Basic', 'CDC', 'DP']

'''
depart_time_list = []
depart_time = 0.0


for i in range(num_veh):
	depart_time_list.append(depart_time)
	random_num = np.random.random()
	delta_time = -1.0 * np.log(random_num) / estimated_arrval_rate
	depart_time += delta_time
'''

totol_benefit_basic = []
totol_benefit_cdc = []
totol_benefit_dp = []


#for d2 in d2_list:
for w_1 in w_1_list:
#for gamma in gamma_list:
#for arrival_rate in arrival_rate_list:

	'''
	arrival_rate = arrival_rate * 0.001
	
	depart_time = 0.0
	depart_time_list = []

	num_veh = int((arrival_rate / 0.01) * 1000)

	for i in range(num_veh):
		depart_time_list.append(depart_time)
		random_num = np.random.random()
		delta_time = -1.0 * np.log(random_num) / arrival_rate
		depart_time += delta_time
	'''

	#gamma = gamma * 0.1
	#print('d2: ', d2)
	print('w_1: ', w_1)
	#print('gamma: ', gamma)
	#print('arrival_rate: ', arrival_rate)
	w_1 = w_1 / 3600.0
	
	for runner_mode in runner_mode_list:

		ini_sk = 0
		sk = 0
		time_interval_list = []


		if runner_mode == 'Basic':
			# Benchmark
			fuel_benefit = 0.0
			time_benefit = 0.0
			totol_benefit = 0.0

			for j in range(len(depart_time_list)-1):
				if (depart_time_list[j+1] - depart_time_list[j]) < 1.5:
					fuel_benefit += eta * k * d2
					time_benefit += 1.5

			totol_benefit = fuel_benefit * w_2 + time_benefit * w_1

			totol_benefit_basic.append(totol_benefit/(len(depart_time_list)-1))
			#totol_benefit_basic.append(totol_benefit/d2 * 1000)

		elif runner_mode == 'CDC':
			# CDC policy
			fuel_benefit = 0.0
			time_benefit = 0.0
			totol_benefit = 0.0

			#print(threshold)
			#threshold = rc.r_calculaton(estimated_arrval_rate, w_1)

			for j in range(len(depart_time_list)-1):
				time_interval = depart_time_list[j+1] - depart_time_list[j]
				sk = ini_sk + time_interval
				
				
				if len(time_interval_list) > 50:
					time_interval_list.pop(0)
				time_interval_list.append(time_interval)

				estimated_arrval_rate = 0
				gamma_index = 0

				for i_pop_list in range(len(time_interval_list)):
					estimated_arrval_rate += time_interval_list[len(time_interval_list) - 1 - i_pop_list] * (gamma_estimator ** gamma_index)
					gamma_index += 1

				estimated_arrval_rate = estimated_arrval_rate * (1.0 - gamma_estimator)
				estimated_arrval_rate = 1.0 / estimated_arrval_rate if estimated_arrval_rate !=0 else 10
				
				threshold = rc.r_calculaton(estimated_arrval_rate, w_1)

				if time_interval <= threshold and (sk < d1/v) and ((d1 / (d1 / v - sk)) < 40.0):
					ini_sk = sk
					fuel_benefit += alpha * d1 * v**2 + eta * k * d2 - alpha * d1 * (d1 / (d1 / v - sk))**2
					time_benefit += sk
				else:
					ini_sk = 0.0

			totol_benefit = fuel_benefit * w_2 + time_benefit * w_1
			totol_benefit_cdc.append(totol_benefit/(len(depart_time_list)-1))
			#totol_benefit_cdc.append(totol_benefit/d2 * 1000)

		elif runner_mode == 'DP':
			# DP Policy
			fuel_benefit = 0.0
			time_benefit = 0.0
			totol_benefit = 0.0

			#print(Z, threshold, C)
			initial_value = [1.0, 25.0, -25.0]
			#Z = tc.theta_c(estimated_arrval_rate, initial_value, w_1, gamma)[0]
			#threshold = tc.theta_c(estimated_arrval_rate, initial_value, w_1, gamma)[1]
			#C = tc.theta_c(estimated_arrval_rate, initial_value, w_1, gamma)[2]

			#print('Z, threshold, C:', Z, threshold, C)

			for j in range(len(depart_time_list)-1):
				time_interval = depart_time_list[j+1] - depart_time_list[j]
				sk = ini_sk + time_interval

				
				if len(time_interval_list) > 50:
					time_interval_list.pop(0)
				time_interval_list.append(time_interval)

				estimated_arrval_rate = 0
				gamma_index = 0

				for i_pop_list in range(len(time_interval_list)):
					estimated_arrval_rate += time_interval_list[len(time_interval_list) - 1 - i_pop_list] * (gamma_estimator ** gamma_index)
					gamma_index += 1

				estimated_arrval_rate = estimated_arrval_rate * (1.0 - gamma_estimator)
				estimated_arrval_rate = 1.0 / estimated_arrval_rate if estimated_arrval_rate !=0 else 10

				#print(estimated_arrval_rate)
				
				Z = tc.theta_c(estimated_arrval_rate, initial_value, w_1, gamma)[0]
				threshold = tc.theta_c(estimated_arrval_rate, initial_value, w_1, gamma)[1]
				C = tc.theta_c(estimated_arrval_rate, initial_value, w_1, gamma)[2]
				initial_value = [Z, threshold, C]

				if sk <= threshold and (sk < d1/v) and ((d1 / (d1 / v - sk)) < 40.0):
					ini_sk = sk
					fuel_benefit += alpha * d1 * v**2 + eta * k * d2 - alpha * d1 * (d1 / (d1 / v - sk))**2
					time_benefit += sk

				else:
					ini_sk = C
					fuel_benefit += alpha * d1 * v**2 - alpha * d1 * (d1 / (d1 / v - C))**2
					time_benefit += C
				#print(sk, fuel_benefit, time_benefit)


			totol_benefit = fuel_benefit * w_2 + time_benefit * w_1
			
			totol_benefit_dp.append(totol_benefit/(len(depart_time_list)-1))
			#totol_benefit_dp.append(totol_benefit/d2 * 1000)



plt.plot(w_1_list, totol_benefit_basic, label='Basic')
plt.plot(w_1_list, totol_benefit_cdc, label='CDC')
plt.plot(w_1_list, totol_benefit_dp, label='DP')

plt.legend(loc='best', fontsize=23)
plt.show()







