from myimport import *
from scipy.spatial import distance
import ast

# original_target_map = {'T_1':'S_1','T_2':'S_2','T_3':'S_3'}
# original_sensor_map = {'S_1':None,'S_2':None,'S_3':None,'S_4':None}
# initialState = Node(original_target_map,original_sensor_map,None,0,1,None)
# print(initialState.target_map)
#

configuration_file = 'monitor.config'
with open(configuration_file,"r") as f:
    config = f.read()
    lines = config.split("\n",3)
    sensors = list(ast.literal_eval(lines[1]))
    print len(sensors[0])
    print sensors
f.close()
a = (1,1)
b = (4,5)
print distance.euclidean(a,b)
