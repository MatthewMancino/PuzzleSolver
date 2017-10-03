import fileinput
import sys
import Queue
import ast
import sets
from scipy.spatial import distance

class Sensor:
    name = None; xLoc = None; yLoc = None; power_remaining = None;
    def __init__(self,name,x,y,po):
        self.name = name; self.xLoc = x; self.yLoc = y; self.power_remaining = po;
    def reducePower(distance):
        print "help"

class Target:
    name = None; xLoc = None; yLoc = None;
    def __init__(self,id,x,y):
        self.name = id; self.xLoc = x; self.yLoc = y
    def updateLoc(self,x,y):
        self.xLoc = x; self.yLoc = y;

class graphNode:
    name = None; xLoc = None; yLoc = None;
    def __init__(self,id,x,y):
        self.name = id; self.xLoc = x; self.yLoc = y

class graphEdge:
    name = None; cost = None; g1 = None; g2 = None;
    def __init__(self,g1,g2,co):
        self.g1 = g1; self.g2 = g2;self.cost = co;


class State:
    target_map = None;
    sensor_map = None;
    def __init__(self,sensors,targets):
        target = {targets[0].name:None,targets[1].name:None,targets[2].name:None}
        sensors = {sensors[0].name:None,sensors[1].name:None,sensors[2].name:None, sensors[3].name:None}


class Node(State):
    visited = False;
    parentNode = None;
    depth = 0;
    pathCost = 0;
    action = None;
    neighbors = None;
    StateHeuristic = None;
    maxT = 0;
    no = 0;

    def __init__(self,o_t,o_s,p_node,dep,pCost):
        self.sensor_map = o_s;self.target_map = o_t;self.parentNode = p_node; self.depth = dep; self.pathCost = pCost;
    def printState():
        print target_map; print sensor_map;
    def neighbor(n):
        neighbors = n;
        return neighbors;
    def maxTime(self):

        sensor_test_map = self.sensor_map.copy()
        sensor_test_keys = self.sensor_map.keys()
        sensor_test_values = self.sensor_map.values()
        target_test_keys = self.target_map.keys()

        global sensors_power
        pRemaining = sensors_power.copy()   #grab the original power level

        x_o = 0
        for t in range(1,max(sensors_power.values())):           #time interval 1 to 1000
            x_o += 1
            for sName in sensor_test_keys:            #for each sensor, subtract the ed at each time interval
                if (sensor_test_map[sName] != None):    #If sensor is pointing to a target
                    pRemaining[sName] = pRemaining[sName] - distance_values[sName+sensor_test_map[sName]]
                if ((pRemaining[sName] <= 0) & (sensor_test_map[sName] != None)): #if the sensor has no power left but is still pointing
                    sensor_test_map[sName] = None;  #remove the sensor from target
                    for tName in target_test_keys:  #if the target is no longer being watched
                        if (tName not in sensor_test_map.values()):
                            self.maxT = x_o;               #End max time calculation

        print "Monitored targets for ",self.maxT,"Seconds"


class dataNode(Node):
    pathList = None;
    unVisited = None;
    active = None;
    def __init__(self,pL,unV,act,pCost,dep,no):
        self.pathList = pL;self.unVisited = unV;self.active = act;self.pathCost = pCost; self.depth = dep;
        self.no = no;



class UndirectedEdge:
    parent_node = None; child_node = None;
    def __init__(self,p,c):
        parent_node = p; child_node = c;


    def getChild(node_no):
        return child_node

    #########

##Global dictionaries giving the remaining power given a sensor and the euclidean distance given a sensor+target string
distance_values = {};
sensors_power = {};

nodes = [];
edges = {};


min_cost_by_depth = [sys.maxint,sys.maxint,sys.maxint,sys.maxint,sys.maxint]
deepest_depth = 1;

#Globals used to keep track of the best state so far given the problems
best_state_config = None;
LongestTime = 0;
smallest = sys.maxint;

############ HELPER FUNCTIONS FOR TARGET-SENSOR PROBLEM
def create_sensors(sense):
    s = [];
    for x in range(0,len(sense)):
        s.append(Sensor(sense[x][0],sense[x][1],sense[x][2],sense[x][3]));
    return s;

def create_target(targ):
    b = []
    for x in range(0,len(targ)):
        b.append(Target(targ[x][0],targ[x][1],targ[x][2]));
    return b;

def calc_ed(targets,sensors):
    z = size(targets);s = size(sensors);i = 0;

############ END HELPERS

########### MAIN SEARCH ALGORITHMS + FUNCTIONS INDEPENDENT OF PROBLEMS

def bfs(initialState,problem):

    explored = sets.Set()
    front = sets.Set()

    frontier = Queue.Queue()
    frontier.put(initialState)
    front.add(str(initialState.sensor_map))

    #While different solutions are still availible
    while (frontier.empty() == False):
        #Get the next potential configuration
        state = frontier.get()

        #Depending on the problem, record the current node's state
        if (problem == "monitor"):
            explored.add(str(state.sensor_map))
        elif(problem == "aggregation"):
            explored.add(str(state.pathList))
        elif(problem == "pancake"):
            explored.add(str(state.pancakeOrder))

        #Test if the current state is a goal state
        if (goal_state(state,problem) == True):
            if (problem == "monitor"):
                print "State",state.sensor_map,"Path Cost",state.pathCost,"Num Action",state.depth
            elif(problem == "aggregation"):
                print "State",state.pathList,state.pathCost,"BFS found goal state"
            elif(problem == "pancake"):
                print "State",state.pathList,state.pathCost,"BFS found goal state"

        #If not, grab all of it's potential neighbors
        SN = neighbor(state,problem)
        for n in SN:
            if (problem == "monitor"):
                if(str(n.sensor_map) not in front):
                    if (str(n.sensor_map) not in explored):
                        front.add(str(n.sensor_map));
                        frontier.put(n);
            elif(problem == "aggregation"):
                if(str(n.pathList) not in front):
                    if (str(n.pathList) not in explored):
                        front.add(str(n.pathList));
                        frontier.put(n);
                        print "PUSH",n.no,"  PathList: ",n.pathList, "unvisited", n.unVisited
            elif(problem == "pancake"):
                explored.add(state.pancakeOrder)


    if (problem == "monitor"):
        if (LongestTime > 0):
            print "Best Config",best_state_config,"Path Cost",state.pathCost,"Depth",state.depth
            print "Cost(Maximum Monitoring Time)", LongestTime
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return True;
        else:
            print "No configuration found"
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return False;
    elif(problem == "aggregation"):
        if (smallest < sys.maxint):
            print "Best Config",best_state_config,"Path Cost",state.pathCost,"Depth",state.depth
            print "Cost(Smallest Time Delay)", smallest
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return True;
        else:
            print "No configuration found"
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return False;
    elif(problem == "pancake"):
        print "State",state.pathList,state.pathCost,"BFS found goal state"


def unicost(initialState, problem):
    # Utilize a FIFO queue for BFS search and separate into two different problem classes

    global lowest_state_config
    global shortest
    explored = sets.Set()
    front = sets.Set()
    frontier = Queue.PriorityQueue()  #Initialize new queue
    frontier.put((initialState.pathCost,initialState))
    front.add(initialState)
    while (frontier.empty() == False):  #While a solution is not found
        state = frontier.get()
        curr = state[1]
        explored.add(curr)
        if (goal_state(curr,problem) == True):
            if (problem == "monitor"):
                print "State",state[1].target_map,state[1].pathCost,state[1].depth, "UCS found new best route"
            elif(problem == "aggregation"):
                print "State",state[1].pathList,state[0], "UCS found new best route"
        elif(problem == "aggregation"):
            if (state[1].pathCost > min_cost_by_depth[len(nodes)-1]):
                break;


        ss = neighbor(curr,problem)
        for x in ss:
            if(x not in front):
                if (x not in explored):
                    #print "Successor",x.pathList
                    front.add(x);
                    frontier.put((x.pathCost,x));

    if (problem == "monitor"):
        if (LongestTime > 0):
            print "Best Config",best_state_config,"Path Cost",state.pathCost,"Depth",state.depth
            print "Cost(Maximum Monitoring Time)", LongestTime
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return True;
        else:
            print "No configuration found"
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return False;
    elif(problem == "aggregation"):
        if (shortest < sys.maxint):
            print "State",state.pathList,state.pathCost,"BFS found goal state"
            print "Cost(Shortest Time Delay)", shortest
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return True;
        else:
            print "No configuration found"
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return False;
    elif(problem == "pancake"):
        if (solved == True):
            print "State",state.pathList,state.pathCost,"BFS found goal state"
        else:
            return False;

def iddfs(initialState,problem):

    global lowest_state_config
    global shortest

    isMore = True;
    maxDepth = 0
    depLimit = 0
    if (problem == "monitor"):
        maxDepth = len(initialState.sensor_map)**len(initialState.target_map)
                #Number of sensors
        print "MAX DEPTH *******",maxDepth,len(initialState.sensor_map),len(initialState.target_map)
        print initialState.sensor_map, initialState.target_map
    elif(problem == "aggregation"):
        maxDepth = len(nodes)
    while (isMore):
        depLimit += 1
        if (depLimit > maxDepth):
            break
        explored = sets.Set()
        front = sets.Set()
        frontier = Queue.LifoQueue()  #Initialize new queue
        frontier.put(initialState)
        front.add(initialState)

        while (frontier.empty() == False):  #While a solution is not found
            state = frontier.get()
            explored.add(state)

            #print "Current", state.pathList
            if (goal_state(state,problem) == True):
                if (problem == "monitor"):
                    print "State",state.target_map,state.pathCost,state.depth
                    print "Time as expressed in nodes: " ,len(front)
                    print "Space -- Frontier:",len(front)," explored:",len(explored)
                    print "Cost(expressed as total nodes searched)", shortest
                    return True;
                elif(problem == "aggregation"):
                    print "State",state.pathList,state.pathCost
            #else:
                #print state.target_map,"Path Cost",state.pathCost,"Depth",state.depth,"     Out of", depLimit

            ss = neighbor(state,problem)
            for x in ss:
                if(x not in front):
                    if (x not in explored):
                        if (x.depth < depLimit):
                            #print "Successor",x.pathList
                            front.add(x);
                            frontier.put(x);
                        else:
                            isMore = True;



    if (problem == "monitor"):
        if (LongestTime > 0):
            print "Best Config",best_state_config,"Path Cost",state.pathCost,"Depth",state.depth
            print "Cost(Maximum Monitoring Time)", LongestTime
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return True;
        else:
            print "No configuration found"
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return False;
    elif(problem == "aggregation"):
        if (shortest < sys.maxint):
            print "State",state.pathList,state.pathCost,"BFS found goal state"
            print "Cost(Shortest Time Delay)", shortest
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return True;
        else:
            print "No configuration found"
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return False;
    elif(problem == "pancake"):
        if (solved == True):
            print "State",state.pathList,state.pathCost,"BFS found goal state"
        else:
            return False;

def greedy(initialState, problem):

    explored = sets.Set()           #New Sets
    front = sets.Set()
    select = Queue.Queue()              #Initialize new queue
    frontier.put(initialState)          #Put initial state
    front.add(initialState)
    while (select.empty() == False):  #While a solution is not found
        state = select.get()
        explored.add(state)
        if (goal_state(state,problem) == True):             #Are we in a goal state?
            if (problem == "monitor"):
                state.pathCost = evaluation_function()
                print "State",state.target_map,state.pathCost, "Greedy found best tMap"
                print "Time as expressed in nodes: " ,len(front)
                print "Space -- Frontier:",len(front)," explored:",len(explored)
                print "Cost(expressed as total nodes searched)", shortest
                return True;
            elif(problem == "aggregation"): #If the problem is an data aggregation problem
                state.pathCost = evaluation_function()
                print "State",state.pathList,state.pathCost, "Greedy found best pathList"

        ss = neighbor(state,problem)                          #Get successors states
        SmallestStateHeuristic = sys.maxint                                               #
        for x in ss:                                                    #Go through every potential state
            if(x not in front):     #If the node is not in the frontier #If the state is listed in the frontier or explored
                if (x not in explored):                                 #And hasn't been explored
                    front.add(x);                                       #Put it in the frontier
                    x.StateHeuristic = evaluation_function(x,problem)          #if the state heuristic is smaller than
                    if (x.StateHeuristic < SmallestStateHeuristic):
                        SmallestStateHeuristic = x.StateHeuristic
                        select.put(x)                    #Only put it in the queue if it is the smallest state heuristic



    if (problem == "monitor"):
        if (LongestTime > 0):
            print "Best Config",best_state_config,"Path Cost",state.pathCost,"Depth",state.depth
            print "Cost(Maximum Monitoring Time)", LongestTime
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return True;
        else:
            print "No configuration found"
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return False;
    elif(problem == "aggregation"):
        if (shortest < sys.maxint):
            print "State",state.pathList,state.pathCost,"BFS found goal state"
            print "Cost(Shortest Time Delay)", shortest
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return True;
        else:
            print "No configuration found"
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return False;
    elif(problem == "pancake"):
        if (solved == True):
            print "State",state.pathList,state.pathCost,"BFS found goal state"
        else:
            return False;



def goal_state(current, problem):
    global best_state_config;
    global LongestTime;

    global smallest;

    global deepest_depth;
    if(problem == "monitor"):           #The goal state for the monitor problem

        t_map = current.target_map      #Is a configuration where the sum of the power loss function
        t_keys = t_map.keys()           #For all four sensors is at a minimums


        for t in t_keys:        #Make sure all targets are covered - If not, no point
            if (t_map[t] == None):
                return False;
        current.maxTime();  #Calculate the max time we can cover these targets
        if (current.maxT > LongestTime):  #If it exceeds the longest record time, set as new record
            LongestTime = current.maxT;
            best_state_config = current.sensor_map;
            return True;
        else:               #Else continue on
            return False;



    elif(problem == "aggregation"):

        if (len(current.unVisited) == 0):
            if(current.pathCost < smallest):
                smallest = current.pathCost
                best_state_config = list(current.pathList)
                return True;
        else:
            return False;

    elif(problem == "pancake"):
        pancake_list = current.pancake_list
        if (pancake_list == pcl):
            return True;
        else:
            return False;

def neighbor(current, problem):
    #Neighbor function acts as an expand function, all this function does is generate
    #All the possible next states given the current state

    successors = sets.Set();

    if(problem == "monitor"):           #For monitor problem
        t_map = current.target_map      #Map target(t) --> sensor
        s_map = current.sensor_map      #Map of sensor(s) --> target
        t_keys = t_map.keys()
        s_keys = s_map.keys()
        global distance_values
        global power_consumption
        for s in s_keys:            #Go through each sensor and target  1,2,3,4
            for t in t_keys:                #Go through each target  1,2,3
                if (s_map[s] != t): #If the current sensor is already watching the target, don't include
                    newS_map = s_map.copy()
                    newT_map = t_map.copy()
                    newS_map[s] = t
                    newT_map[t] = s
                    ed = distance_values[s+t]
                    successors.add(Node(newT_map,newS_map,current,current.depth,current.pathCost + ed))



    elif(problem == "aggregation"):

        pathList = list(current.pathList)  #Get the most recent path list
        unvisited = list(current.unVisited)
        pathCost = current.pathCost
        depth = current.depth
        active = current.active
        no = current.no

        global nodes
        global edges
        print "POP",no,"  PathList: ",pathList, " Unvisited:  ", unvisited

        for y in nodes:
            if(active == None):
                pl = list(pathList);
                uv = list(unvisited);
                pl.append(y[0])
                uv.remove(y)
                ac = y[0];
                dep = depth + 1;
                no += 1
                successors.add(dataNode(pl,uv,ac,0,depth,no));
            elif ((y in unvisited) & (active+y[0] in edges)):
                pl = list(pathList);
                uv = list(unvisited);
                uv.remove(y)
                pl.append(y[0])
                pc = pathCost + edges[active+y[0]];
                ac = y[0];
                depth = depth + 1;
                no += 1
                successors.add(dataNode(pl,uv,ac,pc,depth,no));

    elif(problem == "pancake"):
        print "hi"



    return successors;
def evaluation_function(state, problem):
    if(problem == "monitor"): #For monitor problem, the heuristic would find the
        t_map = state.target_map;                   #Calculates the heurisitic for maximum output


    elif(problem == "aggregation"):
        print "zing"




#### MAIN FUNCTION

def main():

    configuration_file = sys.argv[1]
    algo = sys.argv[2]
    with open(configuration_file,"r") as t:
        text = t.readlines()
    with open(configuration_file,"r") as f:
        config = f.read()

        lines = config.split("\n",1)

        if (lines[0] == "monitor"):
            lines = config.split("\n",3)
        elif (lines[0] == "aggregation"):
            lines = config.split("\n",len(text))
            print len(text)
            print lines[2:len(texr)]


        if (lines[0] == "monitor"):

            original_sensor_map = {}; original_target_map = {};sensor_battery = {}

            sensors = list(ast.literal_eval(lines[1]))
            targets = list(ast.literal_eval(lines[2]))
            s = create_sensors(sensors)
            t = create_target(targets)
            global sensors_power

            #Initialize sensor mappings
            for x in range(0,len(sensors)):
                original_sensor_map[s[x].name] = None;
                sensors_power[s[x].name] = sensors[x][3]

            for y in range(0,len(targets)):
                original_target_map[t[y].name] = None;


            #Find Distance values between all targets and sensors
            for v in range(0,len(sensors)):
                a = (s[v].xLoc,s[v].yLoc)
                for w in range(0,len(targets)):
                    b = (t[w].xLoc,t[w].yLoc)
                    distance_values[s[v].name+t[w].name] = distance.euclidean(a,b)
            initialState = Node(original_target_map,original_sensor_map,None,0,0)
        elif (lines[0] == "aggregation"):

            node_list = list(ast.literal_eval(lines[1]))
            edge_list = []
            for x in lines[2:len()]:
                edge_list.append(list(ast.literal_eval(lines[x])));

            global nodes
            global edges
            for a in node_list: #for each node, append it to the list of nodes
                nodes.append((a[0],a[1],a[2]))

            for b in edge_list:
                edges[b[0]+b[1]] = b[2]
                edges[b[1]+b[0]] = b[2]

            initialState = dataNode([],list(nodes),None,0,1,1)


        elif (lines[0] == "pancakes"):
            pancake_list = list(ast.literal_eval(lines[1]));

    f.close()
    if (algo == "bfs"):
        if (bfs(initialState,lines[0])):
            print "done"
    elif(algo == "ucs"):
        if (unicost(initialState,lines[0])):
            print "done"
    elif(algo == "iddfs"):
        if (iddfs(initialState,lines[0])):
            print "done"

    elif (algo == "greedy"):
        if (greedy(initialState,lines[0])):
            print("Finished")
    # elif (algo == "Astar"):
    #     endState = astar(initialState)
    #     print endState.display()

    return
if __name__ == "__main__":
    main()
