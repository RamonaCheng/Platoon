<!--
 * @Author: WANG Maonan
 * @Date: 2023-03-31 11:46:51
 * @Description: 
 * @LastEditTime: 2023-04-18 17:11:26
-->
- route 有 2 种，时间和距离，
- - short path 是按照距离来进行规划
- - 没有 short path 是按照时间进行规划
platooning 有 3 种，cdc，dp，
所以组合起来一共有 6 种。

- rl_cdc, 估计 travel time, 之前的算法
- rl_dp, 估计 travel time --> rl 去估计时间, 在通过时间来使用数学公式 platooning
- rl_no_platooning, 只做 routing

编队

short 的是只看距离，没有使用 rl 来估计距离
只做 platooning

主要查看 performance：
1. 看一下 6 种方法，在不同 desnsity 下的区别，查看 performance，

下面做一些分析：
1. 分析 rl_dp 的性能
2. 查看合作 policy 的变化（修改 flow，修改路网），说明 RL 可以自适应


- 多个 OD 下，不修改路网，增加车辆数量，测试 rl-dp 和 short-path-dp；先确定 density 下 Q 改变，接着在这个 Q 下测试
- 路网修改距离


## 需要进行的实验

在进行实验之前，需要首先确认一下在指定 density 下 Q 值是否会改变。

- 测试在单个 OD 的情况下，rl-dp 和 short-path-dp 的性能；
- 测试在多个 OD 的情况下，rl-dp 和 short-path-dp 的性能，仿真时间需要长一些；


# 关于 jam density

- 0.005, 0.004, 0.003, 0.002, 0.001


# 修改路网需要修改的内容

## 路网文件的修改

- net 中 edge 的长度；
- add 文件里面探测器的位置

## calculate_d2.py 文件

- calculate_d2.py 文件只有在 rl 的算法里面才会使用
- - d2_link1, 前面 link 的长度
- - return_initial_Qmatrix_Table
- - 1000.0 / 24.0


## theta_calculation.py 文件

- d1 的计算

## r_calculation.py 文件

- d1 的计算

## sumo_env.py 文件

- link_length_list
- if traci.vehicle.getDistance(vehicle) <= 4000.0: 5000-d1
- if distance(self.nodePos,traci.vehicle.getPosition(item))/distance(self.nodePos,self.incLanesOBJ[i].getFromNode().getCoord()) > (d1+100)/(self.incLanesOBJ[i].getLength())


## agent.py 文件，共有 13 个文件

- Qmatrix, __init__, reset
- if d2_distance == 0 or sk > int(d1/24)-2: 
- _f 函数