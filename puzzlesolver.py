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
    def __init__(self,pathList,pCost,act):
        self.pathList = pathList; self.pathCost = pCost; self.action = act;



class UndirectedEdge:
    parent_node = None; child_node = None;
    def __init__(self,p,c):
        parent_node = p; child_node = c;


    def getChild(node_no):
        return child_node

    #########

distance_values = {};nodes = []; edges = {};

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
    explored = sets.Set()
    front = sets.Set()
    frontier = Queue.Queue()  #Initialize new queue
    frontier.put(initialState)
    front.add(initialState)

    while (frontier.empty() == False):  #While a solution is not found
        state = frontier.get()
        explored.add(state)
        if (goal_state(state,problem) == True):
            print("BFS Found a goal state!")
            print("State",state.target_map,state.pathCost)
            print("Time as expressed in nodes: " ,len(front))
            print("Space(expressed as frontier + explored)",len(front),len(explored))
            return True
        ss = successor_function(state,problem)
        for x in ss:
            if(x not in front):
                if (x not in explored):
                    front.add(x);
                    frontier.put(x);


    print("BFS Failed to find a state")
    print("Time as expressed in nodes: " + front.count())
    print("Space(expressed as frontier + explored)" + front + explored)
    print("Cost(expressed as total nodes searched)" + count)
    return False;

def unicost(initialState, problem):
    # Utilize a FIFO queue for BFS search and separate into two different problem classes
    explored = sets.Set()
    front = sets.Set()
    frontier = Queue.PriorityQueue()  #Initialize new queue
    frontier.put((initialState.pathCost,initialState))
    front.add(initialState)
    count = 0;
    while (frontier.empty() == False):  #While a solution is not found
        state = frontier.get()
        curr = state[1]
        print curr
        explored.add(curr)
        count += 1
        if (goal_state(curr,problem) == True):
            print("BFS Found a goal state!")
            print("State",curr.target_map,curr.pathCost)
            print("Time as expressed in nodes: " ,len(front))
            print("Space(expressed as frontier + explored)",len(front),len(explored))
            return True
        ss = successor_function(curr,problem)
        for x in ss:
            print x.pathCost
            if(x not in front):
                if (x not in explored):
                    front.add(x);
                    frontier.put((x.pathCost,x));


    print("UCS Failed to find a state")
    print("Time as expressed in nodes: " + front.count())
    print("Space(expressed as frontier + explored)" + front + explored)
    print("Cost(expressed as total nodes searched)" + count)
    return False;

def goal_state(current, problem):
    if(problem == "monitor"):
        t_map = current.target_map
        t_keys = t_map.keys()
        for t in t_keys:
            if (t_map[t] == None):
                return False;

        print t_map;
        return True;
    elif(problem == "aggregation"):
        if (len(current.pathList) >= len(nodes)-1):
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
            if (t_map[t] == None):  #If target is not being tracked
                for s in s_keys:    #Look for sensor
                    if(s_map[s] == None):   #If sensor is not tracking anything
                        x += 1;
                        newT_map = t_map.copy()
                        newS_map = s_map.copy()
                        newT_map[t] = s
                        newS_map[s] = t
                        action = newT_map[t] + newS_map[s]
                        global distance_values
                        pc = distance_values[action]  #Get Euclidean distance between the sensor and the target
                        successors.add(Node(newT_map,newS_map,current,current.depth + 1,current.pathCost + pc,action))
                        # print("Target being tracked", t_map);


    elif(problem == "aggregation"):
        pathList = current.pathList  #Get the most recent path list

        global nodes
        global edges
        for s in nodes:             #First iteration through the graph
            for y in nodes:             #Second iteration through the graph
                if (y[0] != s[0]):          # If they are the same node, ignore
                    if(y[0]+s[0] in edges):       #If there is an edge between them
                        newPathList = pathList.copy()
                        newPathList.append((s[0],edges[y[0]+s[0]],y[0]));
                        successors.add(dataNode(newPathList,current.pathCost + edges[y[0]+s[0]],y[0]+s[0]));


    return successors;


#### MAIN FUNCTION

def main():

    configuration_file = sys.argv[1]
    algo = sys.argv[2]
    print configuration_file
    with open(configuration_file,"r") as f:
        config = f.read()
        lines = config.split("\n",1)
        if (lines[0] == "monitor"):
            lines = config.split("\n",3)
        elif (lines[0] == "aggregation"):
            lines = config.split("\n",5)
        print len(lines)
        if (lines[0] == "monitor"):

            original_sensor_map = {}; original_target_map = {};

            sensors = list(ast.literal_eval(lines[1]))
            targets = list(ast.literal_eval(lines[2]))
            s = create_sensors(sensors)
            t = create_target(targets)

            #Initialize sensor mappings
            for x in range(0,len(sensors)):
                original_sensor_map[s[x].name] = None;

            #Initialize target mappings
            print original_sensor_map
            for y in range(0,len(targets)):
                original_target_map[t[y].name] = None;

            print original_target_map
            #Find Distance values between all targets and sensors
            for v in range(0,len(sensors)):
                a = (s[v].xLoc,s[v].yLoc)
                for w in range(0,len(targets)):
                    b = (t[w].xLoc,t[w].yLoc)
                    distance_values[s[v].name+t[w].name] = distance.euclidean(a,b)
                    print s[v].name+t[w].name, a+b
            print distance_values
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

            initialState = dataNode([],0,None)


        elif (lines[0] == "pancakes"):
            array = f.readline()
    f.close()
    if (algo == "bfs"):
        if (bfs(initialState,lines[0])):
            print("Finished")
    elif(algo == "ucs"):
        if (unicost(initialState,lines[0])):
            print("Finished")


    # elif (algo == "iddfs"):
    #     endState = iddfs(initialState))
    #     print endState.display()

    # elif (algo == "greedy"):
    #     endState = greedy(initialState)
    #     print endState.display()
    # elif (algo == "Astar"):
    #     endState = astar(initialState)
    #     print endState.display()

    return
if __name__ == "__main__":
    main()
