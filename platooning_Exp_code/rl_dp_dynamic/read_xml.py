import xml.etree.ElementTree as ET
import numpy as np

base_path = '/home/dell/Documents/Research/Network_RL/Platform_Network/platooning/shortest_path_cdc/Nguyen-Dupuis/'
data_path = base_path + 'trip_info.xml'

root = ET.parse(data_path).getroot()

duration_list = []

for type_tag in root.findall('tripinfo'):
	value = type_tag.get('duration')
	duration_list.append(float(value))

print(np.mean(duration_list))