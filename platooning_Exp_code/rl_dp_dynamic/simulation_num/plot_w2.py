import numpy as np
import os
import matplotlib.pyplot as plt
import time

import matplotlib.pyplot as plt

base_path = '/Users/xixiong/Research/Optimization/revision/simulation_num/results/'

ratio_list = np.load(base_path + 'w2.npy')
cdc_list = np.load(base_path + 'cdc.npy')
dp_list = np.load(base_path + 'dp.npy')


ax = plt.subplot(1,1,1)

font1 = {'family': 'Times New Roman', 'weight':'normal', 'size':32}
#font2 = {'family': 'Times New Roman', 'weight':'normal', 'size':23}

plt.plot(ratio_list, cdc_list, label='Policy A', linewidth=4.0, color='darkblue')
plt.plot(ratio_list, dp_list, label='Policy B', linewidth=4.0, color='deeppink')

plt.xlabel('Fuel Price and VOT Ratio', font1)
plt.ylabel('Total Cost Improvement [%]', font1)

ax.spines['left'].set_linewidth(4.5)
ax.spines['right'].set_linewidth(4.5)
ax.spines['bottom'].set_linewidth(4.5)
ax.spines['top'].set_linewidth(4.5)

#plt.xticks(range(24), ['0:00', '1:00', '2:00', '3:00', '4:00', '5:00', '6:00', '7:00', '8:00', '9:00', '10:00', '11:00', '12:00', '13:00', '14:00', '15:00', '16:00', '17:00', '18:00', '19:00', '20:00', '21:00', '22:00', '23:00'])
#plt.xticks(range(24), ['0:00', '', '', '', '4:00', '', '', '', '8:00', '', '', '', '12:00', '', '', '', '16:00', '', '', '', '20:00', '', '', ''])

plt.xticks(fontsize=30)
plt.yticks(fontsize=30)

#plt.title('O-D Pair: %s to %s, Date: June %i, 2013' % (origin, destination, day_index+1), font1)

plt.legend(loc='best', prop=font1, framealpha=1.0)
#plt.legend(loc=(0.7, 0.01), prop=font2, framealpha=1.0)
	
plt.show()


