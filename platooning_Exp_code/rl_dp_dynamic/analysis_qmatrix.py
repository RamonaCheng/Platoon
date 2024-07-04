# 分析 speed
# link18_0, link17_0, link1_0, link2_0, link5_0 的速度变化
import json
import numpy as np
import matplotlib.pyplot as plt

def moving_average(x, w):
    return np.convolve(x, np.ones(w), 'valid') / w


lane_speed_dict = {
    'link1_0': list(),
    'link2_0': list(),
    'link5_0': list(),
    'link17_0': list(),
    'link18_0': list(),
}
times_x = list()
with open('./lane_speed_filter.json', 'r', encoding='utf-8') as f:
    for line in f:
        line_dict = json.loads(line)
        if line_dict['time']>1000 and line_dict['time']<2000:
            times_x.append(line_dict['time']) # 获得时间
            lane_speed = line_dict['speed'] # 获得 speed
            for _lane_id in lane_speed_dict:
                lane_speed_dict[_lane_id].append(lane_speed[_lane_id])


# 根据 csv 文件绘制图像
fig = plt.figure() # step-2
ax1 = fig.add_subplot(511) # row-col-num 
ax2 = fig.add_subplot(512)
ax3 = fig.add_subplot(513)
ax4 = fig.add_subplot(514)
ax5 = fig.add_subplot(515)
fig.tight_layout() # Or equivalently,  "plt.tight_layout()"
axs = {'link1_0':ax1, 'link2_0':ax2, 'link5_0':ax3, 'link17_0':ax4, 'link18_0':ax5}

for _lane_id in ['link1_0', 'link2_0', 'link5_0', 'link17_0', 'link18_0']:
    _y = moving_average(lane_speed_dict[_lane_id], 50)
    axs[_lane_id].plot(times_x[:len(_y)], _y, label=_lane_id)
    axs[_lane_id].legend()

plt.show()