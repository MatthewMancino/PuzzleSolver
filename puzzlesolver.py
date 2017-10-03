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

    def __init__(self,o_t,o_s,p_node,dep,pCost,act):
        self.sensor_map = o_s;self.target_map = o_t;self.parentNode = p_node; self.depth = dep + 1; self.pathCost = pCost; self.action = act;
    def printState():
        print target_map; print sensor_map;
    def neighbor(n):
        neighbors = n;
        return neighbors;


class dataNode(Node):
    pathList = None;
    def __init__(self,pathList,pCost,act,dep):
        self.pathList = pathList; self.pathCost = pCost; self.action = act;self.depth = dep;



class UndirectedEdge:
    parent_node = None; child_node = None;
    def __init__(self,p,c):
        parent_node = p; child_node = c;


    def getChild(node_no):
        return child_node

    #########

distance_values = {};nodes = []; edges = {};
shortest = sys.maxint;
lowest_state_config = None;

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
    # Utilize a FIFO queue for BFS search and separate into two different problem classes
    global lowest_state_config
    global shortest
    explored = sets.Set()
    front = sets.Set()
    frontier = Queue.Queue()  #Initialize new queue
    frontier.put(initialState)
    front.add(initialState)

    while (frontier.empty() == False):  #While a solution is not found
        state = frontier.get()
        explored.add(state)
        print state.pathList
        #print "Current", state.pathList
        if (goal_state(state,problem) == True):
            if (problem == "monitor"):
                print "New smallest function found",state.target_map,state.pathCost
            elif(problem == "aggregation"):
                print "State",state.pathList,state.pathCost,"New smallest path found"

        ss = successor_function(state,problem)
        for x in ss:
            if(x not in front):
                if (x not in explored):
                    front.add(x);
                    frontier.put(x);

    if (shortest != sys.maxint):
        print "State", lowest_state_config,  "BFS found a goal state"
        print "Time as expressed in nodes: " ,len(front)
        print "Space -- Frontier:",len(front)," explored:",len(explored)
        print "Cost(expressed as total nodes searched)", shortest
    else:
        print "BFS Failed to find a state"
        print "State", lowest_state_config
        print "Time as expressed in nodes: " ,len(front)
        print "Space -- Frontier:",len(front)," explored:",len(explored)
        print "Cost(TIME_DELAY)", shortest
    return False;


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
        # print state[1]
        curr = state[1]
        print "Current",curr.pathList
        #print "Current", curr.pathList
        explored.add(curr)
        if (goal_state(curr,problem) == True):
            if (problem == "monitor"):
                print "State",state[1].target_map,state[1].pathCost, "UCS found new best route"
            elif(problem == "aggregation"):
                x = 0
                #print "State",state[1].pathList,state[0], "UCS found new best route"
        ss = successor_function(curr,problem)
        for x in ss:
            if(x not in front):
                if (x not in explored):
                    #print "Successor",x.pathList
                    front.add(x);
                    frontier.put((x.pathCost,x));

    if (shortest != sys.maxint):
        print "State", lowest_state_config, "UCS found a goal state"
        print "Time as expressed in nodes: " ,len(front)
        print "Space -- Frontier:",len(front)," explored:",len(explored)
        print "Cost(TIME DELAY)", shortest
    else:
        print "BFS Failed to find a state"
        print "State", lowest_state_config
        print "Time as expressed in nodes: " ,len(front)
        print "Space -- Frontier:",len(front)," explored:",len(explored)
        print "Cost(expressed as total nodes searched)", shortest
    return False;


def iddfs(initialState,problem):

    global lowest_state_config
    global shortest

    isMore = True;
    depLimit = 0
    if (problem == ""):
        maxDepth = len(initialState.sensor_map) + 1
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
            print state.target_map,state.pathCost
            #print "Current", state.pathList
            if (goal_state(state,problem) == True):
                if (problem == "monitor"):
                    print "State",lowest_state_config
                    print "Time as expressed in nodes: " ,len(front)
                    print "Space -- Frontier:",len(front)," explored:",len(explored)
                    print "Cost(total nodes searched)",shortest
                elif(problem == "aggregation"):
                    print "State",state.pathList,state.pathCost

            ss = successor_function(state,problem)
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
        if (shortest_ed != sys.maxint):
            print "State", lowest_state_config ,"BFS found a goal state"
            print "Time as expressed in nodes: " ,len(front)
            print "Space -- Frontier:",len(front)," explored:",len(explored)
            print "Cost(expressed as total nodes searched)", shortest
    else:
        print "BFS Failed to find a state"
        print "Time as expressed in nodes: " ,len(front)
        print "Space -- Frontier:",len(front)," explored:",len(explored)
        print "Cost(expressed as total nodes searched)", state.pathCost
    return False;


def goal_state(current, problem):
    if(problem == "monitor"):           #The goal state for the monitor problem
        global lowest_state_config;
        global shortest;
        t_map = current.target_map      #Is a configuration where the sum of the power loss function
        t_keys = t_map.keys()           #For all four sensors is at a minimums
        for t in t_keys:
            if (t_map[t] == None):
                return False;
        if (current.pathCost >= shortest):
            return False;
        else:
            shortest = current.pathCost
            lowest_state_config = current.target_map.copy()
            return True;


    elif(problem == "aggregation"):
        if (len(current.pathList) >= len(nodes)-1):
            if(current.pathCost > shortest):
                return False;
            else:
                shortest = current.pathCost
                lowest_state_config = list(current.pathList)
                return True;
        else:
            return False;
    elif(problem == "pancake"):
        pancake_list = current.pancake_list
        if (pancake_list == pcl):
            return True;
        else:
            return False;

def successor_function(current, problem):
    #Create a function that takes the current state and the problem and returns corresponding actions
    #and their mappings
    successors = sets.Set();

    if(problem == "monitor"):
        t_map = current.target_map
        s_map = current.sensor_map
        t_keys = t_map.keys()
        s_keys = s_map.keys()
        x = 0;
        for t in t_keys:
            if (t_map[t] == None):      #If target is not being tracked
                for s in s_keys:
                    if(s_map[s] == None):   #If sensor is not tracking anything
                        x += 1;
                        newT_map = t_map.copy()
                        newS_map = s_map.copy()
                        newT_map[t] = s
                        newS_map[s] = t
                        action = newT_map[t] + newS_map[s] #list the
                        global distance_values
                        pc = distance_values[action]  #Get Euclidean distance between the sensor and the target
                        if (current.pathCost + pc < shortest):
                            successors.add(Node(newT_map,newS_map,current,current.depth,current.pathCost + pc,action))
                        # print("Target being tracked", t_map);


    elif(problem == "aggregation"):
        pathList = current.pathList  #Get the most recent path list

        global nodes
        global edges
        for s in nodes:                     #Go through all the nodes
            for y in nodes:                    #Do it again
                if (y[0] != s[0]):                  # If they are the same node, ignore
                    if(y[0]+s[0] in edges):    #If there is an edge between them
                        newPathList = list(pathList)
                        # print pathList == [];
                        # print y[0]+s[0]
                        # print edges[y[0]+s[0]]
                        if (pathList == []):
                            newPathList.append((y[0],edges[y[0]+s[0]],s[0])); #Node 1 - val - node 2
                            b = dataNode(newPathList,current.pathCost + edges[y[0]+s[0]],y[0]+s[0],current.depth+1)
                            successors.add(b);
                        elif (len(pathList) < len(nodes)-1):
                            for x in pathList:
                                if ((y[0] == x[2]) & (s[0] != x[0])):
                                    c = (y[0],edges[y[0]+s[0]],s[0])
                                    newPathList.append((y[0],edges[y[0]+s[0]],s[0])); #Node 1 - val - node 2
                                    b = dataNode(newPathList,current.pathCost + edges[y[0]+s[0]],y[0]+s[0],current.depth+1)
                                    if (current.pathCost + edges[y[0]+s[0]] < smal):
                                        successors.add(b);
                        elif(len(pathList) >= len(nodes)-1):
                            break;

    elif(problem == "pancake"):
        print "hi"



    return successors;
def evaluation_function(state, problem):
    if(problem == "monitor"): #For monitor problem, the heuristic would be the find the smallest distance
        t_map = state.target_map;



#### MAIN FUNCTION

def main():

    configuration_file = sys.argv[1]
    algo = sys.argv[2]
    with open(configuration_file,"r") as f:
        config = f.read()
        lines = config.split("\n",1)
        if (lines[0] == "monitor"):
            lines = config.split("\n",3)
        elif (lines[0] == "aggregation"):
            lines = config.split("\n",5)


        if (lines[0] == "monitor"):

            original_sensor_map = {}; original_target_map = {};sensor_battery = {}

            sensors = list(ast.literal_eval(lines[1]))
            targets = list(ast.literal_eval(lines[2]))
            s = create_sensors(sensors)
            t = create_target(targets)

            #Initialize sensor mappings
            for x in range(0,len(sensors)):
                original_sensor_map[s[x].name] = None;
                sensor_battery[s[x].name] = sensors[x][3]

            for y in range(0,len(targets)):
                original_target_map[t[y].name] = None;


            #Find Distance values between all targets and sensors
            for v in range(0,len(sensors)):
                a = (s[v].xLoc,s[v].yLoc)
                for w in range(0,len(targets)):
                    b = (t[w].xLoc,t[w].yLoc)
                    distance_values[s[v].name+t[w].name] = distance.euclidean(a,b)
            initialState = Node(original_target_map,original_sensor_map,None,0,0,None)
        elif (lines[0] == "aggregation"):

            node_list = list(ast.literal_eval(lines[1]))
            edge_list = []
            for x in range(2,5):
                edge_list.append(list(ast.literal_eval(lines[x])));

            global nodes
            global edges
            for a in node_list: #for each node, append it to the list of nodes
                nodes.append((a[0],a[1],a[2]))
            for b in edge_list:
                edges[b[0]+b[1]] = b[2]
                edges[b[1]+b[0]] = b[2]

            initialState = dataNode([],0,None,0)


        elif (lines[0] == "pancakes"):
            pancake_list = list(ast.literal_eval(lines[1]));

    f.close()
    if (algo == "bfs"):
        if (bfs(initialState,lines[0])):
            print("Finished")
    elif(algo == "ucs"):
        if (unicost(initialState,lines[0])):
            print("Finished")
    elif(algo == "iddfs"):
        if (iddfs(initialState,lines[0])):
            print("Finished")

    elif (algo == "greedy"):
        if (greedy(initialState,lines[0])):
            print("Finished")
    # elif (algo == "Astar"):
    #     endState = astar(initialState)
    #     print endState.display()

    return
if __name__ == "__main__":
    main()
