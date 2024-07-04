###
 # @Author: WANG Maonan
 # @Date: 2023-03-31 22:03:24
 # @Description: 对 lambda 敏感度的分析
 # nohup bash sensetive_lambda.sh > sensetive_lambda.log &
 # @LastEditTime: 2023-04-14 00:29:37
### 
FOLDER="/home/wangmaonan/traffic/platooning_Exp_2km"

mkdir -p ${FOLDER}/results/sensitive/lambda
for lambda in 0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1
do
    cd ${FOLDER}/rl_dp/
    python -u main.py --jam_density=0.008 --lambda_travel=${lambda} --route=N1_100.rou.xml > ${FOLDER}/results/sensitive/lambda/${lambda}.log &
done