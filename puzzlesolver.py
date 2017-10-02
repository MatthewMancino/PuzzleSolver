import fileinput
import sys
import Queue
import ast
import sets
from myimport import *
from scipy.spatial import distance


distance_values = {};

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

def bfs(initialState,problem,dist):
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
        ss = successor_function(state,problem,dist)
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

    return successors;


#### MAIN FUNCTION

def main():

    nodes = 0;
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
            edges = [5]
            nodes = list(ast.literal_eval(lines[1]))
            for x in range(0,len(lines)):
                print x
                edges[x] = list(ast.literal_eval(lines[x+2]));
            print nodes, edges
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


# def iddfs(initialState):
#     #Use stack because of a DFS subsearch problem
# def astar(initialState):
#     #
# def greedy(initialState):
#     #
