###
 # @Author: WANG Maonan
 # @Date: 2023-03-31 22:03:24
 # @Description: 对 p=w2 敏感度的分析
 # nohup bash sensetive_w1.sh > sensetive_w1.log &
 # conda activate aiolosEnv
 # @LastEditTime: 2023-04-14 00:29:37
### 
FOLDER="/home/wangmaonan/traffic/platooning_Exp_2km"

mkdir -p ${FOLDER}/results/sensitive/w1
for w1 in 5 10 15 20 25 30 35 40 45 50
do
    cd ${FOLDER}/rl_dp/
    sed -i "s/w_1 = [0123456789]\+/w_1 = ${w1}/g" parameters_input.py
    python -u main.py --jam_density=0.008 --route=N2_500.rou.xml > ${FOLDER}/results/sensitive/w1/${w1}.log &
    wait
done