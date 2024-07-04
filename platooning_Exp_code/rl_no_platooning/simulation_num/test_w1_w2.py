import numpy as np
import random
import matplotlib.pyplot as plt
from scipy import integrate
from scipy.optimize import fsolve
import r_calculation as rc
import theta_calculation as tc
import time

# initilizaion
#alpha = 3.51 * 10 ** (-7)
alpha = 3.51 * 10 ** (-7)
d1 = 1000.0
v = 23.0
eta = 0.1
k = 32.2 / 100000.0
d2 = 30000.0
#w_1 = 25.8 / 3600 		# value of time
w_2 = 0.868			# oil price
#w_2 = 0.00000001
gamma_estimator = 0.9
gamma = 0.9
collision_avoidance_time = 2.3
#collision_avoidance_time = 0.0


# define the vehicle flow
upper_branch_arrival_rate_list = [0.071, 0.069, 0.058, 0.057, 0.075, 0.110, 0.193, 0.339, 0.380, 0.317, 0.289, 0.263, 0.279, 0.298, 0.335, 0.348, 0.382, 0.372, 0.375, 0.319, 0.235, 0.207, 0.162, 0.115]
lower_branch_arrival_rate_list = [0.185, 0.146, 0.124, 0.113, 0.191, 0.388, 0.879, 1.527, 1.594, 1.367, 1.191, 1.109, 1.170, 1.209, 1.459, 1.547, 1.586, 1.606, 1.577, 1.386, 1.027, 0.815, 0.620, 0.378]

num_veh = 103825
#CAV_ratio = 0.04
CAV_ratio = 0.04


# define the loop parameters
#w_1_list = range(20, 50, 5)
#w_1_list = range(21.8, 32.7, 10)
w_1_list = np.linspace(10.0, 80.0, num=20)
#CAV_ratio_list = [0.005, 0.01, 0.015, 0.02, 0.025, 0.03, 0.035, 0.04]
#CAV_ratio_list = [6]
w2_w1_list = []

#define the three running mode
runner_mode_list = ['Basic', 'CDC', 'DP']
#runner_mode_list = ['Basic', 'DP']

#define the initial set of 
total_cost_basic = []
total_cost_cdc = []
total_cost_dp = []

fuel_cost_basic = []
fuel_cost_cdc = []
fuel_cost_dp = []

time_cost_basic = []
time_cost_cdc = []
time_cost_dp = []

improve_ratio_cdc = []
improve_ratio_dp = []

#define the fuel rate function
def fuel_rate(vehicle_speed):
	return (3.51 * (10 ** (-4)) * (vehicle_speed ** 3) + 0.407 * vehicle_speed)


# begin the numerical simulation
for w_1 in w_1_list:
	
	print('Value of Time [$/hr]: ', w_1)
	w_1 = w_1 / 3600.0

	w2_w1_list.append(w_2 / w_1)

	# define the departure time of vehicles
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


	# begin the numerical simulation
	for runner_mode in runner_mode_list:

		ini_sk = 0
		sk = 0
		time_interval_list = []

		# the benchmark
		if runner_mode == 'Basic':

			fuel_cost = 0.0
			time_cost = 0.0
			total_cost = 0.0

			# define the cost of the first vehicle
			fuel_cost += (fuel_rate(v) * (d1 / v) + fuel_rate(v) * (d2 / v)) / 1000.0
			time_cost += (d1 / v) + (d2 / v)

			# the cost for other vehicles
			for j in range(len(depart_time_list)-1):
				if (depart_time_list[j+1] - depart_time_list[j]) < collision_avoidance_time:
					fuel_cost += (fuel_rate(v) * (d1 / v) + (fuel_rate(v) * (d2 / v) * (1 - eta))) / 1000.0
					time_cost += (d1 / v) + (d2 / v) - collision_avoidance_time

				else:
					fuel_cost += (fuel_rate(v) * (d1 / v) + (fuel_rate(v) * (d2 / v))) / 1000.0
					time_cost += (d1 / v) + (d2 / v)

			# calculate the total cost
			total_cost = fuel_cost * w_2 + time_cost * w_1

			# append the total cost
			total_cost_basic.append(total_cost / len(depart_time_list))
			fuel_cost_basic.append(fuel_cost / len(depart_time_list))
			time_cost_basic.append(time_cost / len(depart_time_list))

		# the CDC policy
		elif runner_mode == 'CDC':

			fuel_cost = 0.0
			time_cost = 0.0
			total_cost = 0.0

			# leading vehicle speed
			v_leading = v

			# define the cost of the first vehicle
			fuel_cost += (fuel_rate(v) * (d1 / v) + fuel_rate(v) * (d2 / v)) / 1000.0
			time_cost += (d1 / v) + (d2 / v)

			# the cost for other vehicles
			for j in range(len(depart_time_list)-1):
				time_interval = depart_time_list[j+1] - depart_time_list[j]

				# verify the status of the leading vehicle (whether to stay on the track or not)
				leading_vehicle_stay_flag = (depart_time_list[j+1] <= (depart_time_list[j] + d1/v_leading))
				
				# estimate the arrival rate
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
				
				# Calculate the threshold
				threshold = rc.r_calculaton(estimated_arrval_rate, w_1, d2, v)

				#print('threshold: ', threshold)

				# increment of sk
				sk = ini_sk + time_interval

				# catch with leading vehicle
				if time_interval <= threshold and ((d1 / (d1 / v - sk)) < 40.0) and leading_vehicle_stay_flag:
					ini_sk = sk - collision_avoidance_time

					# decide the speed of the following vehicle
					v_following = d1 / (d1 / v - sk + collision_avoidance_time)

					fuel_cost += (fuel_rate(v_following) * (d1 / v - sk + collision_avoidance_time) + fuel_rate(v) * (d2 / v) * (1 - eta)) / 1000.0
					time_cost += (d1 / v - sk + collision_avoidance_time) + (d2 / v)
				
				# not merging with leadning vehicle
				else:
					ini_sk = 0.0

					# decide the speed of the following vehicle
					v_following = v

					fuel_cost += (fuel_rate(v_following) * (d1 / v) + (fuel_rate(v) * (d2 / v))) / 1000.0
					time_cost += (d1 / v) + (d2 / v)

				v_leading = v_following

			total_cost = fuel_cost * w_2 + time_cost * w_1

			# append the total cost
			total_cost_cdc.append(total_cost / len(depart_time_list))
			fuel_cost_cdc.append(fuel_cost / len(depart_time_list))
			time_cost_cdc.append(time_cost / len(depart_time_list))


		# the DP policy
		elif runner_mode == 'DP':

			fuel_cost = 0.0
			time_cost = 0.0
			total_cost = 0.0

			# leading vehicle speed
			v_leading = v

			# define the initial value of threshold and C
			initial_value = [5.0, 15.0, 0.0]

			# define the cost of the first vehicle
			fuel_cost += (fuel_rate(v) * (d1 / v) + fuel_rate(v) * (d2 / v)) / 1000.0
			time_cost += (d1 / v) + (d2 / v)

			# the cost for other vehicles
			for j in range(len(depart_time_list)-1):
				time_interval = depart_time_list[j+1] - depart_time_list[j]
				
				# verify the status of the leading vehicle (whether to stay on the track or not)
				leading_vehicle_stay_flag = (depart_time_list[j+1] <= (depart_time_list[j] + d1/v_leading))
				
				# estimate the arrival rate
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
				
				# Calculate the threshold and C value
				Z = tc.theta_c(estimated_arrval_rate, initial_value, w_1, gamma, d2, v, w_2)[0]
				threshold = tc.theta_c(estimated_arrval_rate, initial_value, w_1, gamma, d2, v, w_2)[1]
				C = tc.theta_c(estimated_arrval_rate, initial_value, w_1, gamma, d2, v, w_2)[2]
				initial_value = [Z, threshold, C]

				#print(initial_value)
				
				'''
				# limit the output of C value
				if abs(C) > 80.0:
					initial_value = [1.0, 25.0, -25.0]
				else:
					initial_value = [Z, threshold, C]
				'''

				# increment of sk
				sk = ini_sk + time_interval

				# catch with leading vehicle
				if sk <= threshold and ((d1 / (d1 / v - sk)) < 40.0) and leading_vehicle_stay_flag:
					ini_sk = sk - collision_avoidance_time

					# decide the speed of the following vehicle
					v_following = d1 / (d1 / v - sk + collision_avoidance_time)

					fuel_cost += (fuel_rate(v_following) * (d1 / v - sk + collision_avoidance_time) + fuel_rate(v) * (d2 / v) * (1 - eta)) / 1000.0					
					time_cost += (d1 / v - sk + collision_avoidance_time) + (d2 / v)
					
				# decelerate with C value
				else:
					ini_sk = C

					# decide the speed of the following vehicle
					v_following = d1 / (d1 / v - C)

					fuel_cost += (fuel_rate(v_following) * (d1 / v - C) + (fuel_rate(v) * (d2 / v))) / 1000.0
					time_cost += (d1 / v - C) + (d2 / v)

			total_cost = fuel_cost * w_2 + time_cost * w_1

			# append the total cost
			total_cost_dp.append(total_cost / len(depart_time_list))
			fuel_cost_dp.append(fuel_cost / len(depart_time_list))
			time_cost_dp.append(time_cost / len(depart_time_list))


#CAV_ratio_list = np.array([0.005, 0.01, 0.015, 0.02, 0.025, 0.03, 0.035, 0.04]) * num_veh



for i in range(len(total_cost_basic)):
	improve_ratio_cdc.append((total_cost_basic[i] - total_cost_cdc[i])/total_cost_basic[i] * 100.0)
	improve_ratio_dp.append((total_cost_basic[i] - total_cost_dp[i])/total_cost_basic[i] * 100.0)

# save total cost improvement
np.save('/Users/xixiong/Research/Optimization/revision/simulation_num/results/w2.npy', w2_w1_list)
np.save('/Users/xixiong/Research/Optimization/revision/simulation_num/results/cdc.npy', improve_ratio_cdc)
np.save('/Users/xixiong/Research/Optimization/revision/simulation_num/results/dp.npy', improve_ratio_dp)

# save travel time 
np.save('/Users/xixiong/Research/Optimization/revision/simulation_num/results/time_basic.npy', time_cost_basic)
np.save('/Users/xixiong/Research/Optimization/revision/simulation_num/results/time_cdc.npy', time_cost_cdc)
np.save('/Users/xixiong/Research/Optimization/revision/simulation_num/results/time_dp.npy', time_cost_dp)


'''
#plt.plot(w_1_list, total_cost_basic, label='Basic')
#plt.plot(w_1_list, total_cost_cdc, label='CDC')
#plt.plot(w_1_list, total_cost_dp, label='DP')

plt.plot(w2_w1_list, time_cost_basic, label='Policy A')
plt.plot(w2_w1_list, time_cost_dp, label='Policy B')

plt.xlabel('Fuel Time Ratio [$/hr]', fontsize=25)
#plt.ylabel('Increased Percentage [%]', fontsize=25)
plt.ylabel('Travel Time [s]', fontsize=25)
plt.xticks(fontsize=25)
plt.yticks(fontsize=23)

plt.legend(loc='best', fontsize=23)
plt.show()
'''






