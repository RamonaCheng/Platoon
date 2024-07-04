###
 # @Author: WANG Maonan
 # @Date: 2024-01-06 00:43:12
 # @Description: 分析 total cost 的组成部分
 # nohup bash run_cost_analysis.sh > run_cost_analysis.log &
 # @LastEditTime: 2024-01-06 01:08:11
### 
FOLDER="$(pwd)"

for exp_name in rl_dp
do
    for j_d in 0.03 0.02 0.019 0.018 0.017 0.016 0.015 0.014 0.013
    do
        cd ${FOLDER}/${exp_name}
        mkdir -p ${FOLDER}/results/multi_cost_analysis/${j_d}/
        python3 -u main.py --jam_density=${j_d} --route=N2_500.rou.xml > ${FOLDER}/results/multi_cost_analysis/${j_d}/${exp_name}.log &
    done
    wait
done


for exp_name in rl_dp
do
    for j_d in 0.012 0.011 0.01 0.009 0.008 0.007 0.006 0.005 0.004
    do
        cd ${FOLDER}/${exp_name}
        mkdir -p ${FOLDER}/results/multi_cost_analysis/${j_d}/
        python3 -u main.py --jam_density=${j_d} --route=N2_500.rou.xml > ${FOLDER}/results/multi_cost_analysis/${j_d}/${exp_name}.log &
    done
    wait
done