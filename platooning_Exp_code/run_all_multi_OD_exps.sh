###
 # @Author: WANG Maonan
 # @Date: 2023-03-31 22:03:24
 # @Description: 同时运行所有的仿真, 并将仿真结果存储在文件中
 # nohup bash run_all_multi_OD_exps.sh > run_all_multi_OD_exps.log&
 # @LastEditTime: 2024-01-06 00:41:29
### 
FOLDER="$(pwd)" #folder = current working directory 

echo "Current working directory: ${FOLDER}"

for j_d in 0.019 0.017 0.015 0.013 0.011
do
    echo "jam_density: ${j_d}"
    for veh_num in 100 300 500
    do
        echo "veh_num: ${veh_num}"
        for exp_name in rl_cdc rl_dp rl_no_platooning short_path_cdc short_path_dp short_path_no_platooning
        do
            echo "exp_name: ${exp_name}"
            cd "${FOLDER}/${exp_name}"
            echo "Changed directory to: $(pwd)"
            mkdir -p "${FOLDER}/results/multi/${j_d}/${veh_num}"
            echo "Created directory: ${FOLDER}/results/multi/${j_d}/${veh_num}"
            python -u main.py --jam_density="${j_d}" --route="N2_${veh_num}.rou.xml" > "${FOLDER}/results/multi_cost_analysis/${j_d}/${veh_num}/${exp_name}.log" &
            echo "Started Python script with parameters: --jam_density=${j_d}, --route=N2_${veh_num}.rou.xml"
        done
        wait
    done
done
