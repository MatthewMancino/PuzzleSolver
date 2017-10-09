import fileinput
import sys
import Queue
import ast
import sets
from scipy.spatial import distance
import math

total_cost_incurred = 0;
data_distances = {};
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
    sumCost = 0;
    a = None;

    def __init__(self,o_t,o_s,dep,pCost,no,act):
        self.sensor_map = o_s;self.target_map = o_t; self.depth = dep; self.pathCost = pCost;
        self.no = no;self.action = act;
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
            for sName in sensor_test_keys:            #for each sensor
                if (sensor_test_map[sName] != None):    #If sensor is pointing to a target, then subtract distance from power remaining
                    pRemaining[sName] = pRemaining[sName] - distance_values[sName+sensor_test_map[sName]]
                if ((pRemaining[sName] <= 0) & (sensor_test_map[sName] != None)): #if the sensor has no power left but is still pointing
                    sensor_test_map[sName] = None;  #remove the sensor from target
                    for tName in target_test_keys:  #if the target is no longer being watched
                        if (tName not in sensor_test_map.values()):
                            self.maxT = x_o;               #End max time calculation

        #print "Monitored targets for ",self.maxT,"Seconds"

        #Path cost is how much an action costs, moving a target would cost
class dataNode(Node):
    pathList = None;
    unVisited = None;
    active = None;
    def __init__(self,pL,unV,act,pCost,dep,no):
        self.pathList = pL;self.unVisited = unV;self.active = act;self.pathCost = pCost; self.depth = dep;
        self.no = no;


##Global dictionaries giving the remaining power given a sensor and the euclidean distance given a sensor+target string
distance_values = {};
sensors_power = {};

nodes = [];
edges = {};
no = 0;
shortest = sys.maxint;


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
    global total_cost_incurred
    explored = sets.Set()
    front = sets.Set()

    front_queue = Queue.Queue()
    front_queue.put(initialState)
    if (problem == "monitor"):
        front.add(str(initialState.sensor_map))
    elif(problem == "aggregation"):
        front.add(str(initialState.pathList))
    elif(problem == "pancake"):
        front.add(str(initialState.pancakeOrder))

    #While different solutions are still availible
    while (front_queue.empty() == False):
        #Get the next potential configuration
        state = front_queue.get()
        total_cost_incurred = total_cost_incurred + state.pathCost


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
                print "State",state.pathList,state.pathCost,"BFS found best route"
            elif(problem == "pancake"):
                print "State",state.pathList,state.pathCost,"BFS found best route"

        #If not, grab all of it's potential neighbors
        SN = neighbor(state,problem)
        for n in SN:
            if (problem == "monitor"):
                if(str(n.sensor_map) not in front):
                    if (str(n.sensor_map) not in explored):
                        front.add(str(n.sensor_map));
                        front_queue.put(n);
                        #print "NEXT",n.no,"  Sendor Config: ",n.sensor_map, "Action Cost", n.pathCost,"ACT:",n.action
            elif(problem == "aggregation"):
                if(str(n.pathList) not in front):
                    if (str(n.pathList) not in explored):
                        front.add(str(n.pathList));
                        front_queue.put(n);
                        #print "PUSH",n.no,"  PathList: ",n.pathList
            elif(problem == "pancake"):
                explored.add(state.pancakeOrder)



    if (problem == "monitor"):
        if (LongestTime > 0):
            print "Best Config",best_state_config
            print "Cost(Maximum Monitoring Time)", LongestTime
            print "Time (In Nodes): " ,len(front)
            print "Space -- front_queue Nodes:",len(front)," Explored Nodes:",len(explored)
            return True;
        else:
            print "No configuration found"
            print "Time (In Nodes): " ,len(front)
            print "Space -- front_queue Nodes:",len(front)," Explored Nodes:",len(explored)
            return False;
    elif(problem == "aggregation"):
        if (smallest < sys.maxint):
            print "Best Config",best_state_config
            print "Cost(Smallest Time Delay)", smallest
            print "Time (In Nodes): " ,len(front)
            print "Space -- front_queue Nodes:",len(front)," Explored Nodes:",len(explored)
            return True;
        else:
            print "No configuration found"
            print "Time (In Nodes): " ,len(front)
            print "Space -- front_queue Nodes:",len(front)," Explored Nodes:",len(explored)
            return False;
    elif(problem == "pancake"):
        print "State",state.pathList,state.pathCost,"BFS found goal state"


def uga(initialState, problem,algo):

    # Utilize a FIFO queue for BFS search and separate into two different problem classes
    explored = sets.Set()                   #The set of all potential states that have been explored
    frontier = sets.Set()                   #The set of all states that have yet to be exploed
    front_queue = Queue.PriorityQueue()     #Additional PQ
    if (algo == "ucs"):
        if (problem == "monitor"):
            front_queue.put((initialState.pathCost,initialState))
        elif (problem == "aggregation"):
            front_queue.put((initialState.sumCost,initialState))
    elif(algo == "greedy"):
        score = evaluation_function(initialState,problem);
        front_queue.put((score,initialState))
    elif(algo == "astar"):
        score = evaluation_function(initialState,problem);
        front_queue.put((score,initialState))


    #The sum of the straight line distances between all sensors and targets is a good

    if (problem == "monitor"):
        frontier.add(str(initialState.sensor_map))
    elif(problem == "aggregation"):
        frontier.add(str(initialState.pathList))
    elif(problem == "pancake"):
        frontier.add(str(initialState.pancakeOrder))

    found = False;

    while (front_queue.empty() == False):      #While there are still states to explore
        st = front_queue.get()
        state = st[1]

        #Add the current state to the set of explored states
        if (problem == "monitor"):
            explored.add(str(state.sensor_map))
        elif(problem == "aggregation"):
            explored.add(str(state.pathList))
        elif(problem == "pancake"):
            explored.add(str(state.pancakeOrder))

        #Test if current state is goal state
        if (goal_state(state,problem) == True):
            if (problem == "monitor"):                  #If it is the goal state
                print "Found new best config"
                found = True;
            elif(problem == "aggregation"):
                print "Found new best route"

        #The following block looks to see if the best possible state has been found, if not expand on the current node.
        if (problem == "monitor"):
            if (algo == "ucs"):
                if (state.maxT < LongestTime):
                    break;
            elif(algo == "greedy"):
                if ((state.a > 0) & found):
                    break;
            elif(algo == "astar"):
                print "dummy"
        elif(problem == "aggregation"):
            if (algo == "ucs"):
                if (state.pathCost < smallest):
                    break;
            elif(algo == "greedy"):
                if ((state.a > 0) & found):
                    break;
            elif(algo == "astar"):
                print "dummy"
        elif(problem == "pancake"):
            print "nada"

        #Expand the current node, aka look for additional states
        SN = neighbor(state,problem)
        for n in SN:
            if (problem == "monitor"):
                curr_config = str(n.sensor_map);
                if((curr_config not in frontier) & (curr_config not in explored)):
                    front_queue.add(ma);
                    if(algo == "ucs"):
                        n.a = 0
                        frontier.put((n.pathCost,n));
                    elif(algo == "greedy"):
                        n.a = evaluation_function(n,problem)
                        front_queue.put((n.a,n));
                    elif(algo == "astar"):
                        n.a = evaluation_function(n,problem)
                        front_queue.put((n.a+n.pathCost,n));

                    #Debug statement
                    print "PUSH",n.no,"  Sensor Config: ",n.sensor_map, "ACT:", n.pathCost,"AC:", n.action,n.a,n.a+n.pathCost
            elif(problem == "aggregation"):
                if(str(n.pathList) not in front):
                    if (str(n.pathList) not in explored):
                        front.add(str(n.pathList));
                        if(algo == "ucs"):
                            front_queue.put((n.pathCost,n));
                        elif(algo == "greedy"):
                            a = evaluation_function(n,problem)
                            front_queue.put((a,n));
                        elif(algo == "astar"):
                            a = evaluation_function(n,problem)
                            front_queue.put((a+n.pathCost,n));
                        #print "PUSH",n.no," Path List: ",n.pathList, "AC:", n.pathCost,"ACT:", n.action, "EVAL", a, a+n.pathCost
            elif(problem == "pancake"):
                explored.add(state.pancakeOrder)


    if (problem == "monitor"):
        if (LongestTime > 0):
            print "Best Config",best_state_config
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
            print "Best Config",best_state_config
            print "Cost(Shortest Time Delay)", smallest
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return True;
        else:
            print "No configuration found"
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
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
    elif(problem == "aggregation"):
        maxDepth = len(nodes)+15
                #Number of sensors
        print "MAX DEPTH *******",maxDepth
    while (isMore):
        depLimit += 1
        if (depLimit > maxDepth):
            break
        explored = sets.Set()
        front = sets.Set()
        front_queue = Queue.LifoQueue()  #Initialize new queue
        front_queue.put(initialState)
        front.add(initialState)

        while (front_queue.empty() == False):  #While a solution is not found
            state = front_queue.get()
            explored.add(state)

            # print "Current", state.pathList
            if (goal_state(state,problem) == True):
                if (problem == "monitor"):
                    print "State",state.target_map,state.pathCost,state.depth
                    print "Time as expressed in nodes: " ,len(front)
                    print "Space -- Frontier:",len(front)," explored:",len(explored)
                    print "Cost(Max Monitoring)", LongestTime
                    return True;
                elif(problem == "aggregation"):
                    #print "State",state.pathList,state.pathCost
                    a = 0
            #else:
                #print state.target_map,"Path Cost",state.pathCost,"Depth",state.depth,"     Out of", depLimit
            # if (problem == "aggregation"):
            #     if (state.depth > deepest_depth):
            #         if (state.pathCost > smallest):
            #             isMore = False
            #             break


            ss = neighbor(state,problem)
            for x in ss:
                if(x not in front):
                    if (x not in explored):
                        if (x.depth < depLimit):
                            #print "Successor",x.pathList
                            front.add(x);
                            front_queue.put(x);
                        else:
                            isMore = True;



    if (problem == "monitor"):
        if (LongestTime < 0):
            print "Best Config",best_state_config,"Path Cost",state.pathCost,"Depth",state.depth
            print "Cost(Maximum Monitoring Time)", LongestTime * -1
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
            print "State",state.pathList,state.pathCost,"BFS found goal state"
            print "Cost(Shortest Time Delay)", smallest
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return True;
        else:
            print "No configuration found"
            print "Time (In Nodes): " ,len(front)
            print "Space -- Frontier Nodes:",len(front)," Explored NOdes:",len(explored)
            return False;



def goal_state(current, problem):
    global best_state_config;
    global LongestTime;

    global smallest;
    global total_cost_incurred
    global shortest;

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
            total_cost_incurred = current.sumCost;
            return True;
        else:               #Else continue on
            return False;

    elif(problem == "aggregation"):

        if (len(current.unVisited) == 0):
            if(current.pathCost < smallest):
                smallest = current.pathCost
                deepest_depth = current.depth
                best_state_config = list(current.pathList)
                return True;
        else:
            return False;



def neighbor(current, problem):
    #Neighbor function acts as an expand function, all this function does is generate
    #All the possible next states given the current state

    successors = sets.Set();
    global no
    if(problem == "monitor"):
        t_map = current.target_map      #Map target(t) --> sensor
        s_map = current.sensor_map      #Map of sensor(s) --> target
        t_keys = t_map.keys()
        s_keys = s_map.keys()
        #print "CURR",current.no,"  PathList: ",s_map,"SUM", current.sumCost, current.a
        global distance_values
        global total_cost_incurred
        global power_consumption        # 1->1,2,3,4
        for t in t_keys:
            for s in s_keys:            #Go through each sensor
                if (t_map[t] == None): #Only if sensor is not assigned
                    if(s_map[s] == None):
                        newS_map = s_map.copy()
                        newT_map = t_map.copy()
                        newS_map[s] = t
                        newT_map[t] = s
                        action = s+t
                        no += 1
                        a = Node(newT_map,newS_map,current.depth,distance_values[s+t],no,action)
                        a.sumCost = current.sumCost + distance_values[s+t]
                        successors.add(a)



    elif(problem == "aggregation"):

        pathList = list(current.pathList)  #Get the most recent path list
        unvisited = list(current.unVisited)
        pathCost = current.pathCost
        depth = current.depth
        active = current.active

        global nodes
        global edges
        #print "POP",current.no,"  PathList: ",pathList, " Unvisited:  ", unvisited, "PC", pathCost

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

    return successors;
def evaluation_function(state, problem):
    global nodes
    global data_distances
    if(problem == "monitor"): #For monitor problem, the heuristic would find the
        t_map = state.target_map
        t_keys = t_map.keys()
        x = 0;
        for key in t_keys:
            if (t_map[key] == None):
                x += 1;
        return x;
        #The states with the fewest missing targets will be next
    elif(problem == "aggregation"):
        #Greedy function is the configuration with the smallest total distance from eachotgher\
        for x in nodes:
            if (state.active != None):
                if (state.active+x[0] in edges):
                    return data_distances[state.active+x[0]];
#### MAIN FUNCTION

def main():

    global no
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

            no += 1
            initialState = Node(original_target_map,original_sensor_map,0,0,no,None)
        elif (lines[0] == "aggregation"):

            node_list = list(ast.literal_eval(lines[1]))
            edge_list = []
            for x in lines[2:len(text)]:
                edge_list.append(list(ast.literal_eval(x)));

            global nodes
            global edges

            for a in node_list: #for each node, append it to the list of nodes
                nodes.append((a[0],a[1],a[2]))

            for y in nodes:
                for x in nodes:
                    if (x != y):
                        data_distances[x[0]+y[0]] = distance.euclidean((x[1],x[2]),(y[1],y[2]))
            #print data_distances

            for b in edge_list:
                edges[b[0]+b[1]] = b[2]
                edges[b[1]+b[0]] = b[2]
            no += 1
            initialState = dataNode([],list(nodes),None,0,1,no)


        elif (lines[0] == "pancakes"):
            pancake_list = list(ast.literal_eval(lines[1]));

    f.close()
    if (algo == "bfs"):
        if (bfs(initialState,lines[0])):
            a = 0
    elif(algo == "iddfs"):
        if (iddfs(initialState,lines[0])):
            a = 0
    elif((algo == "ucs") | (algo == "greedy") | (algo == "astar")):
        if (uga(initialState,lines[0],algo)):
            a = 0

    # elif (algo == "astar"):
    #     endState = astar(initialState)
    #     print endState.display()

    return
if __name__ == "__main__":
    main()
