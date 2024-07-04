import numpy as np
import os
import matplotlib.pyplot as plt
import time

import matplotlib.pyplot as plt

upper_branch = [254, 249, 210, 206, 269, 397, 693, 1221, 1367, 1141, 1040, 946, 1005, 1074, 1205, 1251, 1374, 1340, 1351, 1150, 845, 745, 582, 413]
lower_branch = [665, 525, 445, 407, 687, 1398, 3164, 5498, 5740, 4922, 4286, 3993, 4212, 4351, 5252, 5568, 5711, 5783, 5677, 4989, 3696, 2934, 2233, 1361]


ax = plt.subplot(1,1,1)

font1 = {'family': 'Times New Roman', 'weight':'normal', 'size':30}
#font2 = {'family': 'Times New Roman', 'weight':'normal', 'size':23}

plt.plot(upper_branch, label='Freeway I-210', linewidth=2.5)
plt.plot(lower_branch, label='Freeway 134', linewidth=2.5)

plt.xlabel('Departure Time Interval', font1)
plt.ylabel('Flow [veh / hour]', font1)

ax.spines['left'].set_linewidth(2.5)
ax.spines['right'].set_linewidth(2.5)
ax.spines['bottom'].set_linewidth(2.5)
ax.spines['top'].set_linewidth(2.5)

#plt.xticks(range(24), ['0:00', '1:00', '2:00', '3:00', '4:00', '5:00', '6:00', '7:00', '8:00', '9:00', '10:00', '11:00', '12:00', '13:00', '14:00', '15:00', '16:00', '17:00', '18:00', '19:00', '20:00', '21:00', '22:00', '23:00'])
plt.xticks(range(24), ['0:00', '', '', '', '4:00', '', '', '', '8:00', '', '', '', '12:00', '', '', '', '16:00', '', '', '', '20:00', '', '', ''])

plt.xticks(fontsize=25)
plt.yticks(fontsize=25)

#plt.title('O-D Pair: %s to %s, Date: June %i, 2013' % (origin, destination, day_index+1), font1)

plt.legend(loc='best', prop=font1, framealpha=1.0)
#plt.legend(loc=(0.7, 0.01), prop=font2, framealpha=1.0)
	
plt.show()


