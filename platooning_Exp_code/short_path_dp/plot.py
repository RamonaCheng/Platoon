import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

a1 = np.load('q1.npy')
a4 = np.load('q4.npy')
a5 = np.load('q5.npy')
a6 = np.load('q6.npy')
a7 = np.load('q7.npy')
a8 = np.load('q8.npy')
a9 = np.load('q9.npy')
a10 = np.load('q10.npy')
a11 = np.load('q11.npy')
a12 = np.load('q12.npy')
a13 = np.load('q13.npy')

fig, ax = plt.subplots(nrows=3, ncols=4)
time_window = 10 + 1



ax[0, 0].plot(a1[:,0], savgol_filter(a1[:,1], time_window, 3), 'tab:blue')
ax[0, 1].plot(a4[:,0], savgol_filter(a4[:,1], time_window, 3), 'tab:orange')
ax[0, 2].plot(a5[:,0], savgol_filter(a5[:,1], time_window, 3), 'tab:green')
ax[0, 3].plot(a6[:,0], savgol_filter(a6[:,1], time_window, 3), 'tab:red')
ax[1, 0].plot(a7[:,0], savgol_filter(a7[:,1], time_window, 3), 'tab:purple')
ax[1, 1].plot(a8[:,0], savgol_filter(a8[:,1], time_window, 3), 'tab:brown')
ax[1, 2].plot(a9[:,0], savgol_filter(a9[:,1], time_window, 3), 'tab:pink')
ax[1, 3].plot(a10[:,0], savgol_filter(a10[:,1], time_window, 3), 'tab:gray')
ax[2, 0].plot(a11[:,0], savgol_filter(a11[:,1], time_window, 3), 'tab:olive')
ax[2, 1].plot(a12[:,0], savgol_filter(a12[:,1], time_window, 3), 'tab:cyan')
ax[2, 2].plot(a13[:,0], savgol_filter(a13[:,1], time_window, 3), 'yellowgreen')

ax[0, 0].set_title('Junction 1')
ax[0, 1].set_title('Junction 4')
ax[0, 2].set_title('Junction 5')
ax[0, 3].set_title('Junction 6')
ax[1, 0].set_title('Junction 7')
ax[1, 1].set_title('Junction 8')
ax[1, 2].set_title('Junction 9')
ax[1, 3].set_title('Junction 10')
ax[2, 0].set_title('Junction 11')
ax[2, 1].set_title('Junction 12')
ax[2, 2].set_title('Junction 13')


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
print('Junction 4 data length: ', len(a4))
print('Junction 5 data length: ', len(a5))
print('Junction 6 data length: ', len(a6))
print('Junction 7 data length: ', len(a7))
print('Junction 8 data length: ', len(a8))
print('Junction 9 data length: ', len(a9))
print('Junction 10 data length: ', len(a10))
print('Junction 11 data length: ', len(a11))
print('Junction 12 data length: ', len(a12))
print('Junction 13 data length: ', len(a13))