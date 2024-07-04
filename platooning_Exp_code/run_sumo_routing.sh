###
 # @Author: WANG Maonan
 # @Date: 2023-03-31 22:03:24
 # @Description: 测试使用 sumo route 和 no platooning
 # nohup bash run_sumo_routing.sh > run_sumo_routing.log &
 # @LastEditTime: 2023-11-16 20:28:15
### 
FOLDER="$(pwd)"

for exp_name in sumo_routing_no_platooning sumo_routing_cdc
do

    for j_d in 0.03 0.02 0.019 0.018 0.017 0.016 0.015 0.014 0.013
    do
        cd ${FOLDER}/${exp_name}
        mkdir -p ${FOLDER}/results/${j_d}/
        python -u main.py --jam_density=${j_d} --route=N2_500.rou.xml > ${FOLDER}/results/${j_d}/${exp_name}.log &
    done
    wait
done


for exp_name in sumo_routing_no_platooning sumo_routing_cdc
do
    for j_d in 0.012 0.011 0.01 0.009 0.008 0.007 0.006 0.005 0.004
    do
        cd ${FOLDER}/${exp_name}
        mkdir -p ${FOLDER}/results/${j_d}/
        python -u main.py --jam_density=${j_d} --route=N2_500.rou.xml > ${FOLDER}/results/${j_d}/${exp_name}.log &
    done
    wait
done