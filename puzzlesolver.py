import fileinput
import sys
import Queue

def main():

    configuration_file = sys.argv[1]
    algorithm_used = sys.argv[2]
    print configuration_file
    with open(configuration_file,"r") as f:
        config = f.read()
        lines = config.split("\n",3)
        if (lines[0] == "monitor"):
            sensor_array = f.readline()
        elif (lines[0] == "aggregation"):

    f.close()
    if (algo == "bfs"):
        bfs()
    elif algo == "iddfs"):
        ucs_solve()
    elif(algo == "ucs"):
        iddfs_solve()
    elif (algo == "greedy"):
    elif (algo == "Astar"):

    return
if __name__ == "__main__":
    main()

def bfs(nodes, edges):
    # Utilize a fifo queue for BFS search and separate into two different problem classes
    q = Queue()
    start_state = nodes[0]


def goal_test():

def get_successive_states():

def ucs_solve(nodes, edges):
    # Utilize Priority Queue with weights on the Euclidean distance between two points



def iddfs_solve(nodes, edges):
    #Use stack because of a DFS subsearch problem


class Sensor:
    name = None; x_loc = None; y_loc = None; power_remaining = None;
    def __init__(self,name,x,y,po):
        self.name = name; self.x_loc = x; self.y_loc = y; self.power_remaining = po;

class InfraredBeam:
    sensor_name = None; target_name = None; distance_to_target = None;
    def changeTarget(t_name):
        target_name = t_name

class Target:
    name = None; x_loc = None; y_loc = None;
    def __init__(self,id,x,y):
        self.name = id; self.x_loc = x; self.y_loc = y
    def updateLoc(self,x,y):
        self.x_loc = x; self.y_loc = y;

class Node:
    """This is a Python docstring"""
    nodeName = None; xLoc = None; yLoc = None;
    def __init__(self):
        self.data = []
    def setNextNode(y):
        nextNode = y
        return y;
class UndirectedEdge:
    time_delay = None; node_one = None; node_two = None;

    def __init__(self,time,e1,e2):
        time_delay = time; edge_one = e1; edge_two = e2;

    def getOtherEdge(node_no):
        if (node_no == node_one):
            return node_two
        else if (node_no == node_two)
            return node_one
