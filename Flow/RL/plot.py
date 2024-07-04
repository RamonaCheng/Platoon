import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

a1 = np.load('q1.npy')
a2 = np.load('q2.npy')
a3 = np.load('q3.npy')
a4 = np.load('q4.npy')
a5 = np.load('q5.npy')

fig, ax = plt.subplots(nrows=2, ncols=3)
time_window = 10 + 1



ax[0, 0].plot(a1[:,0], savgol_filter(a1[:,1], time_window, 3), 'tab:blue')
ax[0, 1].plot(a2[:,0], savgol_filter(a2[:,1], time_window, 3), 'tab:orange')
ax[0, 2].plot(a3[:,0], savgol_filter(a3[:,1], time_window, 3), 'tab:green')
ax[1, 0].plot(a4[:,0], savgol_filter(a4[:,1], time_window, 3), 'tab:red')
ax[1, 1].plot(a5[:,0], savgol_filter(a5[:,1], time_window, 3), 'tab:purple')


ax[0, 0].set_title('Junction 1')
ax[0, 1].set_title('Junction 2')
ax[0, 2].set_title('Junction 3')
ax[1, 0].set_title('Junction 4')
ax[1, 1].set_title('Junction 5')


'''
plt.plot(a1[:,0], savgol_filter(a1[:,1], time_window, 3), 'tab:blue')
plt.plot(a4[:,0], savgol_filter(a4[:,1], time_window, 3), 'tab:orange')

plt.plot(a5[:,0], savgol_filter(a5[:,1], time_window, 3), 'tab:green')
plt.plot(a6[:,0], savgol_filter(a6[:,1], time_window, 3), 'tab:red')
plt.plot(a7[:,0], savgol_filter(a7[:,1], time_window, 3), 'tab:purple')
plt.plot(a8[:,0], savgol_filter(a8[:,1], time_window, 3), 'tab:brown')
plt.plot(a9[:,0], savgol_filter(a9[:,1], time_window, 3), 'tab:pink')
plt.plot(a10[:,0], savgol_filter(a10[:,1], time_window, 3), 'tab:gray')
plt.plot(a11[:,0], savgol_filter(a11[:,1], time_window, 3), 'tab:olive')
plt.plot(a12[:,0], savgol_filter(a12[:,1], time_window, 3), 'tab:cyan')
plt.plot(a13[:,0], savgol_filter(a13[:,1], time_window, 3), 'yellowgreen')
'''

#plt.legend()
plt.show()


print('Junction 1 data length: ', len(a1))
print('Junction 2 data length: ', len(a2))
print('Junction 3 data length: ', len(a3))
print('Junction 4 data length: ', len(a4))
print('Junction 5 data length: ', len(a5))