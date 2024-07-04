import numpy as np
from random import random

def generate_routefile():

	# define the parameters of the flow, 07:00 - 08:00; 08:00 - 09:00; 09:00 - 10:00. 
	#upper_branch_arrival_rate_list = [0.127, 0.127, 0.127]
	#lower_branch_arrival_rate_list = [0.127, 0.194, 0.167]

	arrival_rate_list = [0.06]

	routes=open("./Nguyen-Dupuis/N.rou.xml", "w")
	print("""<?xml version="1.0" encoding="UTF-8"?>

<routes>

	<route id="route0" edges="start1 end1" />
    <route id="route1" edges="start1 end2" />
    <route id="route2" edges="start2 end1" />
    <route id="route3" edges="start2 end2" />


    <vType id="conventional" vClass="passenger" tau="5.5" minGap="7.5" carFollowModel="IDM" maxSpeed="40.00"/>
    <vType id="conventional_human" vClass="passenger" tau="5.5" minGap="7.5" carFollowModel="IDM" maxSpeed="40.00" color="52,128,235"/>
    <!--vType id="conventional_human" vClass="passenger" tau="3.5" minGap="5.5" carFollowModel="IDM" maxSpeed="28.00" color="52,128,235"/-->
    <vType id="non_merging" vClass="passenger" tau="5.5" minGap="7.5" carFollowModel="IDM" maxSpeed="40.00"/>
    <vType id="merging" vClass="passenger" carFollowModel="IDM" emergencyDecel="4.5" length="5" tau="0.1" maxSpeed="40.00" color="1,1,0" accel="10.3" decel="1.7" sigma="0.0" minGap="2.0"/>
    <!--vType id="connected" vClass="passenger" tau="2.5" minGap="5.5" carFollowModel="IDM" maxSpeed="28.00" lcCooperative="0.0" color="1,1,0" lcStrategic="1.0" accel="3.0" decel="1.5" emergencyDecel="4.5" lcSpeedGain="0.3" sigma="0.0" lcKeepRight="0.3"-->
    <vType id="connected" vClass="passenger" tau="5.5" minGap="7.5" carFollowModel="IDM" maxSpeed="1.00" emergencyDecel="4.5"/>
    <vType id="routeByDistance" maxSpeed="1"/>
    <vType id="connected_pCatchupFollower" vClass="passenger" carFollowModel="IDM" emergencyDecel="4.5" length="5" tau="0.1" maxSpeed="40.00" color="1,1,0" accel="10.3" decel="1.7" sigma="0.0" minGap="2.0"/>
    <vType id="connected_pCatchup" vClass="passenger" tau="0.1" carFollowModel="IDM" emergencyDecel="4.5" maxSpeed="40.00" color="0,0.8,0.3" accel="10.3" decel="1.7" sigma="0.0" minGap="2.0"/>
    <vType id="connected_pFollower" vClass="passenger" tau="0.1" carFollowModel="IDM" maxSpeed="40.00" color="1,1,0" accel="10.3" decel="1.7" sigma="0.0"  minGap="2.0" emergencyDecel="4.5"/>
    <vType id="connected_pLeader" vClass="passenger" length="5" tau="7.5" minGap="9.5" carFollowModel="IDM" maxSpeed="40.00" lcCooperative="0.0" color="1,1,0" lcStrategic="1.0" accel="2.0" decel="1.5" emergencyDecel="4.5" lcSpeedGain="0.3" sigma="0.0" lcKeepRight="0.3"/>
    
	""", file=routes)

	#generate the vehicles for exponential distribution
	def generateDepartTime(arrival_rate_list, num_veh, delay_index):
		if delay_index == True:
			depart_time = 10.0
		else:
			depart_time = 0.0

		depart_time_list = []
		for i in range(num_veh):
			arrival_rate = arrival_rate_list[0]

			depart_time_list.append(depart_time)
			random_num = np.random.random_sample()
			delta_time = -1.0 * np.log(random_num) / arrival_rate
			depart_time += delta_time
		
		return depart_time_list
	
	class Trip:
		def __init__(self,depart_time,fromLoc,toLoc,sequenceNum):
			self.depart_time=depart_time
			self.fromLoc=fromLoc
			self.toLoc=toLoc
			self.sequenceNum=sequenceNum
		def __eq__(self,other):
			return self.depart_time==other.depart_time
		def __gt__(self,other):
			return self.depart_time>other.depart_time
		def __lt__(self,other):
			return other>self
		def __repr__(self):
			return ('    <trip id="%sTo%s_%i" type="conventional"  from="%s" to="%s" depart="%f" />' % (self.fromLoc,self.toLoc,self.sequenceNum,self.fromLoc,self.toLoc,self.depart_time))

	class Trip_Normal:
		def __init__(self,depart_time,fromLoc,toLoc,sequenceNum):
			self.depart_time=depart_time
			self.fromLoc=fromLoc
			self.toLoc=toLoc
			self.sequenceNum=sequenceNum
		def __eq__(self,other):
			return self.depart_time==other.depart_time
		def __gt__(self,other):
			return self.depart_time>other.depart_time
		def __lt__(self,other):
			return other>self
		def __repr__(self):
			return ('    <trip id="%sTo%s_%i" type="conventional_human" from="%s" to="%s" depart="%f" />' % (self.fromLoc,self.toLoc,self.sequenceNum,self.fromLoc,self.toLoc,self.depart_time))

	def collectFiles(depart_time_list,routes,fromLoc,toLoc,allTrips, HV_ratio):
		for j in range(0, len(depart_time_list)):
			random_num = random()
			if random_num >= HV_ratio:
				allTrips.append(Trip(depart_time_list[j],fromLoc,toLoc,j))	# the case for CAVs
			else:
				allTrips.append(Trip_Normal(depart_time_list[j],fromLoc,toLoc,j))	# the case for HDVs
			#print('    <trip id="%sTo%s_%i" vType="connected" color="1,1,0" from="%s" to="%s" depart="%f" />' % (fromLoc,toLoc,j,fromLoc,toLoc,depart_time_list[j]), file=routes)
				
	allTrips=[]
	# for i in range(1,3):
	# 	for j in range(1,3):
	# 		depart_time_list=generateDepartTime(upper_branch_arrival_rate_list,num_veh,CAV_ratio)
	# 		fromLoc="start"+str(i)
	# 		toLoc="end"+str(j)
	# 		collectFiles(depart_time_list,routes,fromLoc,toLoc,allTrips)

	depart_time_list=generateDepartTime(arrival_rate_list,500, False)
	fromLoc="start1"
	toLoc="end1"
	HV_ratio = 0.0
	collectFiles(depart_time_list,routes,fromLoc,toLoc,allTrips, HV_ratio)

	depart_time_list=generateDepartTime(arrival_rate_list,500, False)
	fromLoc="start2"
	toLoc="end1"
	HV_ratio = 0.0
	collectFiles(depart_time_list,routes,fromLoc,toLoc,allTrips, HV_ratio)

	depart_time_list=generateDepartTime(arrival_rate_list,500, True)
	fromLoc="start1"
	toLoc="end2"
	HV_ratio = 0.0
	collectFiles(depart_time_list,routes,fromLoc,toLoc,allTrips, HV_ratio)

	depart_time_list=generateDepartTime(arrival_rate_list,500, True)
	fromLoc="start2"
	#fromLoc="link4"
	toLoc="end2"
	HV_ratio = 0.0
	collectFiles(depart_time_list,routes,fromLoc,toLoc,allTrips, HV_ratio)

	allTrips.sort()
	print('***********************Number of generated vehicles: ', len(allTrips))
	
	for trip in allTrips:
		print(trip,file=routes)
	
	print("</routes>", file=routes)
	routes.close()


	# write the file of the detecors
	# with open("data/detector.xml", "w") as detector:

	# 	print('''<?xml version="1.0" encoding="UTF-8"?>
	# 	<additional xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/additional_file.xsd">

	# 	<e1Detector id="coordinator_0" lane="input_1_0" pos="500.0" freq="900.00" file="coordinator_0.xml"/>
	# 	<e1Detector id="coordinator_1" lane="input_2_0" pos="500.0" freq="900.00" file="coordinator_0.xml"/>

	# 	''', file=detector)

	# 	print("</additional>", file=detector)	
	