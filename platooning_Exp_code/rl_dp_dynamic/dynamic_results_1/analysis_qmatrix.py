# 分析 Q matrix
import json

with open('./Qmatirx_table.json', 'r', encoding='utf-8') as f:
    for line in f:
        line_dict = json.loads(line)
        print(line_dict['time']) # 获得仿真时间
        print(line_dict['Q']) # 获得 Q-value