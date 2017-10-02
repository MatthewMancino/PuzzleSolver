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



class UndirectedEdge:
    parent_node = None; child_node = None;
    def __init__(self,p,c):
        parent_node = p; child_node = c;


    def getChild(node_no):
        return child_node
