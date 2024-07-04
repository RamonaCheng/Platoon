'''
@Author: WANG Maonan
@Date: 2023-04-13 23:31:38
@Description: SUMO 环境
@LastEditTime: 2023-11-15 16:27:32
'''
import os
import sys

try:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
except KeyError:
    raise Exception("Please set the 'SUMO_HOME' environment variable.")

import sumolib
import traci
import simpla
import numpy as np
import dijkstar

#import theta_calculation as tc
import queue
from random import random
from dijkstar import Graph, find_path

import parameters_input as para

# read nominal parameters
LEARNING_RATE, MAX_MEMORY, BATCH_SIZE, GAMMA, gamma, w_1, w_2, d1, collision_time_delay = para.nominal_values()
	
def distance(coord1,coord2):
    return (abs(coord1[0]-coord2[0])**2+abs(coord1[1]-coord2[1])**2)**0.5
class lane:
    def __init__(self,ID):
        self.ID=ID
        self.time_interval_list=[]
        self.lead_time=0
        self.flow=0

    def reset(self):
        self.time_interval_list.clear()
        self.lead_time=0
        self.flow=0

class junction:
    def __init__(self, ID, incomingLanes,outgoingLanes, nodePosition, jam_density=0.0095):
        self.ID = ID
        self.incLanes = []
        self.incLanesOBJ=[]
        self.outLanes = []
        self.inflows={}
        self.inflow_rates={}
        self.outflows={}
        self.outflow_rates={}
        for lane in incomingLanes:
            laneid=lane.getID()
            self.incLanes.append(laneid)
            self.incLanesOBJ.append(lane)
            self.inflows[laneid]=queue.Queue(50)
            self.inflow_rates[laneid]=0
        for lane in outgoingLanes:
            laneid=lane.getID()
            self.outLanes.append(laneid)
            self.outflows[laneid]=queue.Queue(50)
            self.outflow_rates[laneid]=0
        self.nodePos=nodePosition
        self.lead_vehicle = None
        self.onLaneVehicles=[]
        self.lead_time = 0
        self.temp_vehicle = []
        self.temp_time = 0

        self.link_length_list = np.array([
                                    2.0, 2.0, 2.0, 2.8, 2.0, 
                                    2.0, 2.0, 2.0, 2.0, 2.0, 
                                    2.0, 2.0, 2.8, 2.0, 2.0, 
                                    2.0, 2.0, 4.5, 2.0]
                                ) * 1000.0
        self.jam_density = jam_density
        
    
    def getTk(self):
        # calculate the predicted headway sk
        temp_time=traci.simulation.getTime()
        return temp_time

    def detectArrival(self):
        detected_vehicles = []
        for lane in self.incLanes:
            if "link" in lane:
                # should be for loop here
                if len(traci.inductionloop.getLastStepVehicleIDs("e1Detector"+lane)) > 0:
                    for loop_vehicle_1 in traci.inductionloop.getLastStepVehicleIDs("e1Detector"+lane):
                        loop_vehicle_type = traci.vehicle.getTypeID(loop_vehicle_1)
                        if loop_vehicle_type in ['connected_pFollower', 'connected_pLeader', 'conventional', 'connected_pLeader_restricted']:
                        #if loop_vehicle_type in ['conventional_human']:
                            detected_vehicles.append(loop_vehicle_1)
            elif 'start1' in lane:
                if len(traci.inductionloop.getLastStepVehicleIDs("e1Detectorstart1")) > 0:
                    for loop_vehicle_2 in traci.inductionloop.getLastStepVehicleIDs("e1Detector"+lane):
                        loop_vehicle_type = traci.vehicle.getTypeID(loop_vehicle_2)
                        if loop_vehicle_type in ['connected_pFollower', 'connected_pLeader', 'conventional', 'connected_pLeader_restricted']:
                        #if loop_vehicle_type in ['conventional_human']:
                            detected_vehicles.append(loop_vehicle_2)
            elif 'start2' in lane:
                if len(traci.inductionloop.getLastStepVehicleIDs("e1Detectorstart2")) > 0:
                    for loop_vehicle_3 in traci.inductionloop.getLastStepVehicleIDs("e1Detector"+lane):
                        loop_vehicle_type = traci.vehicle.getTypeID(loop_vehicle_3)
                        if loop_vehicle_type in ['connected_pFollower', 'connected_pLeader', 'conventional', 'connected_pLeader_restricted']:
                        #if loop_vehicle_type in ['conventional_human']:
                            detected_vehicles.append(loop_vehicle_3)
                    
        return detected_vehicles



    def restrictDrivingMode(self):
        for i in range(len(self.incLanes)):
            for item in traci.edge.getLastStepVehicleIDs(self.incLanes[i]):
                #if distance(self.nodePos,traci.vehicle.getPosition(item))/distance(self.nodePos,self.incLanesOBJ[i].getFromNode().getCoord()) > 1100/(self.incLanesOBJ[i].getLength()):
                if 1:
                    #traci.vehicle.setSpeedMode(item, 1)
                    #traci.vehicle.setSpeed(item, 24.0)

                    vehicle_type = traci.vehicle.getTypeID(item)

                    if vehicle_type == 'conventional':
                        traci.vehicle.setSpeedMode(item, 0)
                        #traci.vehicle.setSpeed(item, 24.0)
                    elif vehicle_type == 'merging':
                        traci.vehicle.setSpeedMode(item, 1)
                        traci.vehicle.setType(item, 'connected_pFollower')
                    elif vehicle_type == 'non_merging':
                        traci.vehicle.setSpeedMode(item, 1)
                        traci.vehicle.setType(item, 'connected_pLeader')
                    elif vehicle_type == 'connected_pFollower':
                        traci.vehicle.setSpeedMode(item, 1)
                        traci.vehicle.setType(item, 'connected_pFollower')
                    elif vehicle_type == 'connected_pLeader':
                        traci.vehicle.setSpeedMode(item, 1)
                        traci.vehicle.setType(item, 'connected_pLeader')

                    traci.vehicle.setSpeed(item, 24.0)

                    # simulate the congestion effect
                    if self.incLanes[i] not in ['start1', 'start2']:
                        link_name = self.incLanes[i]
                        link_count = int(link_name.split('k')[1]) - 1
                        link_length = self.link_length_list[link_count]

                        density = traci.edge.getLastStepVehicleNumber(self.incLanes[i]) / link_length
                        # print(density, density / self.jam_density)
                        speed_limit = max(24.0 * (1.0 - density / self.jam_density), 4.0)

                        if vehicle_type == 'connected_pLeader':
                            traci.vehicle.setSpeed(item, speed_limit)


class network:
    def __init__(self,path,ui,sumocfgPath,routePath,steptime=500, jam_density=0.0095):
        net=sumolib.net.readNet(path)
        self.junctions=[]
        self.lanes={}
        self.ui=ui
        self.routePath = routePath
        self.sumocfgPath = sumocfgPath
        for node in net.getNodes():
            if "junction" in node.getID() and node.getID()[9:] not in ["10","12"]:
                self.junctions.append(
                        junction(
                            ID = node.getID(),
                            incomingLanes = node.getIncoming(),
                            outgoingLanes = node.getOutgoing(),
                            nodePosition= node.getCoord(),
                            jam_density=jam_density
                        ))
                
        # generate vehicle departures
        #gr.generate_routefile()

        # define detected information, including arrival time and vehicle IDs
        self.detected_junction = []
        self.detected_vehicles = []
        self.detected_time = []
        self.detected_fuel = []
        self.detected_link_time = []    # calculate the interval time during the link

        # define the vehicle total cost
        self.vehicle_fuel = {}
        self.vehicle_time = {}

        self.non_cav_list = []
        self.cav_list = []

        self.link_list = np.array([7.0, 9.0, 9.0, 12.0, 3.0, 9.0, 5.0, 13.0, 5.0, 9.0, 9.0, 10.0, 9.0, 6.0, 9.0, 8.0, 7.0, 14.0, 11.0]) * 1000.0


        self.cost_cav = 0.0
        self.cost_non_cav = 0.0

        for edge in net.getEdges():
            if "link" in edge.getID():
                self.lanes[edge.getID()]=lane(edge.getID())
        
        self.steptime=steptime

    def Dijkstar(self, origin, destination):
        graph = Graph()
        graph.add_edge(1, 5, self.link_list[0])       #link 1
        graph.add_edge(1, 12, self.link_list[1])      #link 2
        graph.add_edge(4, 5, self.link_list[2])      #link 3
        graph.add_edge(4, 9, self.link_list[3])      #link 4
        graph.add_edge(5, 6, self.link_list[4])      #link 5
        graph.add_edge(5, 9, self.link_list[5])      #link 6
        graph.add_edge(6, 7, self.link_list[6])      #link 7
        graph.add_edge(6, 10, self.link_list[7])      #link 8
        graph.add_edge(7, 8, self.link_list[8])      #link 9
        graph.add_edge(7, 11, self.link_list[9])      #link 10
        graph.add_edge(8, 2, self.link_list[10])      #link 11
        graph.add_edge(9, 10, self.link_list[11])      #link 12
        graph.add_edge(9, 13, self.link_list[12])      #link 13
        graph.add_edge(10, 11, self.link_list[13])      #link 14
        graph.add_edge(11, 2, self.link_list[14])      #link 15
        graph.add_edge(11, 3, self.link_list[15])      #link 16
        graph.add_edge(12, 6, self.link_list[16])      #link 17
        graph.add_edge(12, 8, self.link_list[17])      #link 18
        graph.add_edge(13, 3, self.link_list[18])      #link 19

        #return find_path(graph, origin, destination).total_cost
        return find_path(graph, origin, destination)

    def observe(self):

        traci.simulationStep()
        
        '''
        print('Link 1 mean speed: ', traci.lane.getLastStepMeanSpeed('link1_0'))
        print('Link 2 mean speed: ', traci.lane.getLastStepMeanSpeed('link2_0'))
        print('Link 3 mean speed: ', traci.lane.getLastStepMeanSpeed('link3_0'))
        print('Link 4 mean speed: ', traci.lane.getLastStepMeanSpeed('link4_0'))
        print('Link 5 mean speed: ', traci.lane.getLastStepMeanSpeed('link5_0'))
        print('Link 6 mean speed: ', traci.lane.getLastStepMeanSpeed('link6_0'))
        print('Link 7 mean speed: ', traci.lane.getLastStepMeanSpeed('link7_0'))
        print('Link 8 mean speed: ', traci.lane.getLastStepMeanSpeed('link8_0'))
        print('Link 9 mean speed: ', traci.lane.getLastStepMeanSpeed('link9_0'))
        print('Link 10 mean speed: ', traci.lane.getLastStepMeanSpeed('link10_0'))
        print('Link 11 mean speed: ', traci.lane.getLastStepMeanSpeed('link11_0'))
        print('Link 12 mean speed: ', traci.lane.getLastStepMeanSpeed('link12_0'))
        print('Link 13 mean speed: ', traci.lane.getLastStepMeanSpeed('link13_0'))
        print('Link 14 mean speed: ', traci.lane.getLastStepMeanSpeed('link14_0'))
        print('Link 15 mean speed: ', traci.lane.getLastStepMeanSpeed('link15_0'))
        print('Link 16 mean speed: ', traci.lane.getLastStepMeanSpeed('link16_0'))
        print('Link 17 mean speed: ', traci.lane.getLastStepMeanSpeed('link17_0'))
        print('Link 18 mean speed: ', traci.lane.getLastStepMeanSpeed('link18_0'))
        print('Link 19 mean speed: ', traci.lane.getLastStepMeanSpeed('link19_0'))
        '''
        
        # change the vehicle color and record the fuel
        for vehicle in traci.vehicle.getIDList():
            vehicle_item_type = traci.vehicle.getTypeID(vehicle)

            if (vehicle_item_type == 'connected_pFollower' or vehicle_item_type == 'connected_pCatchup' or vehicle_item_type == 'connected_pCatchupFollower'):
                traci.vehicle.setColor(vehicle,(0,255,0))   # Green
            elif vehicle_item_type == 'conventional':
                traci.vehicle.setColor(vehicle,(255,255,0))     # yellow
            elif vehicle_item_type == 'connected_pLeader':
                traci.vehicle.setColor(vehicle,(255,0,100))     # purple
            elif vehicle_item_type == 'merging':
                traci.vehicle.setColor(vehicle,(0,255,255))
            elif vehicle_item_type == 'non_merging':            
                traci.vehicle.setColor(vehicle,(255,255,255))   # white

            
            '''
            # restrict the speed at the junction for vehicle level coordination
            if traci.vehicle.getRoadID(vehicle).split('_')[0] == ':junction5':
                traci.vehicle.setSpeedMode(vehicle, 0)
                traci.vehicle.setSpeed(vehicle, 24.0)
            '''
            
            # define the vehicle fuel consumption and travel time
            if traci.vehicle.getDistance(vehicle) <= 5000-d1:
                self.vehicle_fuel[vehicle] = 0.0
                self.vehicle_time[vehicle] = 0.0
            else:
                vehicle_speed = traci.vehicle.getSpeed(vehicle)
                if (vehicle_item_type == 'connected_pFollower' or vehicle_item_type == 'connected_pCatchup' or vehicle_item_type == 'connected_pCatchupFollower'):
                    fuel_rate = 3.51 * (10 ** (-4)) * (vehicle_speed ** 3) + 0.407 * vehicle_speed  # ml/s
                    fuel_rate = fuel_rate * 0.9

                    time_rate = traci.simulation.getDeltaT()    # seconds
                    time_rate = time_rate * 1.0
                else:
                    fuel_rate = 3.51 * (10 ** (-4)) * (vehicle_speed ** 3) + 0.407 * vehicle_speed  # ml/s

                    time_rate = traci.simulation.getDeltaT()    # seconds

                self.vehicle_fuel[vehicle] += fuel_rate * traci.simulation.getDeltaT() # mL
                self.vehicle_time[vehicle] += time_rate

            #print(vehicle, self.vehicle_fuel[vehicle], self.vehicle_time[vehicle])

        '''
        # simulate the traffic accident
        current_time = traci.simulation.getTime()

        # simulate the first accident
        if current_time > 1000 and current_time < 5000:
            for item in traci.edge.getLastStepVehicleIDs('link5'):
                vehicle_item_type = traci.vehicle.getTypeID(item)

                if vehicle_item_type == 'connected_pLeader':
                    traci.vehicle.setType(item, 'connected_pLeader_restricted')

        elif current_time > 5000:
            for item in traci.edge.getLastStepVehicleIDs('link5'):
                vehicle_item_type = traci.vehicle.getTypeID(item)

                if vehicle_item_type == 'connected_pLeader':
                    traci.vehicle.setType(item, 'connected_pLeader')

        # simulate the second accident
        if current_time > 1000 and current_time < 5000:
            for item in traci.edge.getLastStepVehicleIDs('link13'):
                vehicle_item_type = traci.vehicle.getTypeID(item)

                if vehicle_item_type == 'connected_pLeader':
                    traci.vehicle.setType(item, 'connected_pLeader_restricted')

        elif current_time > 5000:
            for item in traci.edge.getLastStepVehicleIDs('link13'):
                vehicle_item_type = traci.vehicle.getTypeID(item)

                if vehicle_item_type == 'connected_pLeader':
                    traci.vehicle.setType(item, 'connected_pLeader')

        # simulate the third accident
        if current_time > 1000 and current_time < 5000:
            for item in traci.edge.getLastStepVehicleIDs('link9'):
                vehicle_item_type = traci.vehicle.getTypeID(item)

                if vehicle_item_type == 'connected_pLeader':
                    traci.vehicle.setType(item, 'connected_pLeader_restricted')

        elif current_time > 5000:
            for item in traci.edge.getLastStepVehicleIDs('link9'):
                vehicle_item_type = traci.vehicle.getTypeID(item)

                if vehicle_item_type == 'connected_pLeader':
                    traci.vehicle.setType(item, 'connected_pLeader')

        # simulate the fourth accident
        if current_time > 1000 and current_time < 5000:
            for item in traci.edge.getLastStepVehicleIDs('link10'):
                vehicle_item_type = traci.vehicle.getTypeID(item)

                if vehicle_item_type == 'connected_pLeader':
                    traci.vehicle.setType(item, 'connected_pLeader_restricted')

        elif current_time > 5000:
            for item in traci.edge.getLastStepVehicleIDs('link10'):
                vehicle_item_type = traci.vehicle.getTypeID(item)

                if vehicle_item_type == 'connected_pLeader':
                    traci.vehicle.setType(item, 'connected_pLeader')
        '''

        # record non-cav id when entering the network
        entering_vehicles = traci.simulation.getDepartedIDList()
        for entering_item in entering_vehicles:
            # record the vehicle entering the system
            entering_item_type = traci.vehicle.getTypeID(entering_item)
            if entering_item_type == 'conventional_human':
                self.non_cav_list.append(entering_item)
            else:
                self.cav_list.append(entering_item)

                '''
                # define shortest path ratio, other vehicles will route according to A-star routing algorithm
                shortest_path_ratio = 0.0
                random_num = random()
                if random_num > (1.0 - shortest_path_ratio):
                    # set shorest route to vehicle
                    current_link = traci.vehicle.getRoadID(entering_item)
                    fromEdge = current_link
                    route_links = traci.vehicle.getRoute(entering_item)
                    toEdge = route_links[-1]
                    stageResult = traci.simulation.findRoute(fromEdge, toEdge, 'routeByDistance')

                    traci.vehicle.setRoute(entering_item, stageResult.edges)
                '''

        # Record Non-CAV total cost
        for left_vehicle in traci.simulation.getArrivedIDList():
            if left_vehicle in self.non_cav_list:
                self.non_cav_list.remove(left_vehicle)
                self.cost_non_cav += (self.vehicle_fuel[left_vehicle]/1000.0 * w_2 + self.vehicle_time[left_vehicle]/3600.0 * w_1)
            else:
                self.cav_list.remove(left_vehicle)
                #print('left cav cost: ', (self.vehicle_fuel[left_vehicle]/1000.0 * w_2 + self.vehicle_time[left_vehicle]/3600.0 * w_1))
                self.cost_cav += (self.vehicle_fuel[left_vehicle]/1000.0 * w_2 + self.vehicle_time[left_vehicle]/3600.0 * w_1)
            
            self.vehicle_fuel.pop(left_vehicle)
            self.vehicle_time.pop(left_vehicle)


        # return detected information
        self.detected_junction = []
        self.detected_vehicles = []
        self.detected_time = []
        self.detected_fuel = []
        self.detected_link_time = []


        for i in range(len(self.junctions)):
            # restrict the driving mode
            self.junctions[i].restrictDrivingMode()

            # if we detect the vehicle arrival, report the arrival time and vehicle ids
            observed_info = self.junctions[i].detectArrival()

            if len(observed_info) > 0:
                # avoid the case where vehicles at the same junction arrives at simutaneously
                for observed_junction_vehicle in observed_info:
                    self.detected_junction.append(i)
                    self.detected_vehicles.append(observed_junction_vehicle)
                    self.detected_time.append(traci.simulation.getTime())
                    self.detected_fuel.append(self.vehicle_fuel[observed_junction_vehicle])
                    self.detected_link_time.append(self.vehicle_time[observed_junction_vehicle])


        # arrange the observation
        observation_junction = self.detected_junction
        observation_vehicle = self.detected_vehicles
        observation_time = self.detected_time
        observation_fuel = self.detected_fuel
        observation_link_time = self.detected_link_time
        
        return observation_junction, observation_vehicle, observation_time, observation_fuel, observation_link_time


    def reset(self):
        traci.close()
        traci.start(
            [
                sumolib.checkBinary(self.ui), 
                '-c', os.path.join(self.sumocfgPath),
                '-r', self.routePath
            ]
        )
        simpla.load("data/simpla.cfg.xml")
        for junction in self.junctions:
            for lane in self.lanes:
                self.lanes[lane].reset()
        
        observation_junction = []
        observation_vehicle = []
        observation_time = []
        observation_fuel = []
        observation_link_time = []

        # define the vehicle total cost
        self.vehicle_fuel = {}
        self.vehicle_time = {}

        self.non_cav_list = []
        self.cav_list = []

        self.cost_cav = 0.0
        self.cost_non_cav = 0.0


        return observation_junction, observation_vehicle, observation_time, observation_fuel, observation_link_time

    def return_cost_non_cav(self):
        return self.cost_non_cav

    def return_cost_cav(self):
        return self.cost_cav

    def close(self):
        traci.close()