###
 # @Author: WANG Maonan
 # @Date: 2023-03-31 22:03:24
 # @Description: 同时运行所有的仿真, 并将仿真结果存储在文件中
 # nohup bash run_all_single_OD_exps.sh > run_all_single_OD_exps.log &
 # @LastEditTime: 2023-04-14 00:29:37
### 
FOLDER="$(pwd)"

for j_d in 0.003 0.005 0.007 0.01 0.02
do
    for exp_name in rl_dp short_path_dp
    do
        for veh_num in 100 300
        do
            cd ${FOLDER}/${exp_name}
            mkdir -p ${FOLDER}/results/single/${j_d}/${veh_num}
            python -u main.py --jam_density=${j_d} --route=N1_${veh_num}.rou.xml > ${FOLDER}/results/single/${j_d}/${veh_num}/${exp_name}.log &
        done
    done
    wait
done