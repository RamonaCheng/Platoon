###
 # @Author: WANG Maonan
 # @Date: 2023-03-31 22:03:24
 # @Description: 对 arrival rate 敏感度的分析
 # nohup bash sensetive_arrival_rate.sh > sensetive_arrival_rate.log &
 # conda activate aiolosEnv
 # @LastEditTime: 2023-04-14 00:29:37
### 
FOLDER="/home/wangmaonan/traffic/platooning_Exp_2km"

mkdir -p ${FOLDER}/results/sensitive/ar
for ar in 02 04 06 08 10 12 14 16 18 20
do
    cd ${FOLDER}/rl_dp/
    python -u main.py --jam_density=0.008 --route=N2_500_0${ar}.rou.xml > ${FOLDER}/results/sensitive/ar/${ar}.log &
done