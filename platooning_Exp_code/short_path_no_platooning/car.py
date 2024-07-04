'''
@Author: WANG Maonan
@Date: 2023-03-31 10:29:09
@Description: 
@LastEditTime: 2023-03-31 21:32:51
'''
# import simpy

def car(env):
	while True:
		print('Start parking at %d' % env.now)
		parking_duration = 5
		yield env.timeout(parking_duration)
		print('Start driving at %d' % env.now)
		trip_duration = 2
		yield env.timeout(trip_duration)