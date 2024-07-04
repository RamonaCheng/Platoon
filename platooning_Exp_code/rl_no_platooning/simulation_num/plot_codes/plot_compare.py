import numpy as np
import os
import matplotlib.pyplot as plt
import time

import matplotlib.pyplot as plt

new_id_list = ['001', '002', '003', '004', '005', '006', '007', '07A', '008', '08A', '009', '010', '011', '012', '013', '13A', '014', '14A', '14B', '14C', '15E', '15W', '15X', '16E', '16W', '18W']

'''
# The OD pair with high flow
high_index = 591
origin = new_id_list[high_index//25]
destination = new_id_list[high_index%25]
'''

origin = '014'
destination = '15X'


origin_index = new_id_list.index(origin)
destination_index = new_id_list.index(destination)

selected_id = origin_index * 25 + destination_index

gnn_od_path = '/home/xi/Documents/Research/OD_prediction/DL_prediction/data/new_gnn_npy/results/three_step/FL-GCN-CNN/step_3/'
real_od_path = '/home/xi/Documents/Research/OD_prediction/DL_prediction/data/new_gnn_npy/test/od_label_npy/'
kf_od_path = '/home/xi/Documents/Research/OD_prediction/DL_prediction/data/new_kf/results/final_kf_3_step/'
his_od_path = '/home/xi/Documents/Research/OD_prediction/DL_prediction/data/new_gnn_npy/test/his_od_label_npy/'

final_files = sorted(os.listdir(gnn_od_path))


if 1:
#for day_index in range(2, 30):
	day_index = 2

	gnn_od = []
	kf_od = []
	real_od = []
	his_od = []

	for item in final_files:

		item_index = final_files.index(item)

		if (item_index >= day_index * 12)  and (item_index <= (day_index+1) * 12 - 1):		# see one day results
		#if 1:		#see one month results

			# load neural networks data
			gnn_od_data = np.reshape(np.load(gnn_od_path+item), 650).tolist()

			gnn_od.append(gnn_od_data[selected_id])

			#load kf data
			kf_od_data = np.load(kf_od_path+item)[selected_id][0]

			if kf_od_data < 0.0:
				kf_od_data = 0.0

			kf_od.append(kf_od_data)

			#load his od data
			his_od_data = np.reshape(np.load(his_od_path+item), 650).tolist()
			his_od.append(his_od_data[selected_id])

			# load real od data
			real_data = np.reshape(np.load(real_od_path+item), 650).tolist()
			#print(real_data.index(np.array(real_data).max()))
			#time.sleep(2.0)
			real_od.append(real_data[selected_id])

	#print(gnn_od, real_od)
	#print(np.array(gnn_od).shape)
	ax = plt.subplot(1,1,1)

	font1 = {'family': 'Times New Roman', 'weight':'normal', 'size':30}
	#font2 = {'family': 'Times New Roman', 'weight':'normal', 'size':23}

	plt.plot(gnn_od, label='FL-GCN', linewidth=2.5)
	plt.plot(kf_od, label='Kalman filter', linewidth=2.5)
	plt.plot(real_od, '-', color='black', label='True OD', linewidth=2.5)
	plt.plot(his_od, '--', color='black', label='Historical OD', linewidth=2.5)

	plt.xlabel('Departure Time Interval', font1)
	plt.ylabel('Flow [veh / 15 min]', font1)

	ax.spines['left'].set_linewidth(2.5)
	ax.spines['right'].set_linewidth(2.5)
	ax.spines['bottom'].set_linewidth(2.5)
	ax.spines['top'].set_linewidth(2.5)

	#plt.xticks(range(15), ['6:15 am', '', '6:45 am', '', '7:15 am', '', '7:45 am', '', '8:15 am', '', '8:45 am', '', '9:15 am', '', '9:45 am'])
	#plt.xticks(range(14), ['6:30 am', '', '7:00 am', '', '7:30 am', '', '8:00 am', '', '8:30 am', '', '9:00 am', '', '9:30 am'])
	plt.xticks(range(13), ['6:45 am', '', '7:15 am', '', '7:45 am', '', '8:15 am', '', '8:45 am', '', '9:15 am', '', '9:45 am'])

	plt.xticks(fontsize=25)
	plt.yticks(fontsize=25)

	plt.title('O-D Pair: %s to %s, Date: June %i, 2013' % (origin, destination, day_index+1), font1)

	plt.legend(loc='best', prop=font1, framealpha=1.0)
	#plt.legend(loc=(0.7, 0.01), prop=font2, framealpha=1.0)
	plt.show()


