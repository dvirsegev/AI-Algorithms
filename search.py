import csv
from ways import load_map_from_csv
import ways.info as info
from ways.tools import compute_distance
import heapq
import math
from ways import tools

#import random
# GLOBAL
RESULTUSC = 'results/UCSRuns.txt'
RESULTASTAR= 'results/AStarRuns.txt'
RESULTIDA= 'results/IDAStar.txt'
PROBLEMS = "problems.csv"
TABLE_COST = {}
new_limit = 0

# This class represent the Junction.
class State:
    def __init__(self, junction):
        self.index = junction.index
        self.links = junction.links
        self.parents = None
        self.lat = junction.lat
        self.lon = junction.lon

    def __lt__(self, other):
        return TABLE_COST[self.index] < TABLE_COST[other.index]


    # return all the neighbors of the node- type NODE
    def get_neightbors(self, roads):
        list_neightbors = []
        for link in self.links:
            # set new state of each row
            node = State(roads[getattr(link,"target")])
            list_neightbors.append(node)
            TABLE_COST[node.index] = 0
        return list_neightbors

    # write into the file, could be ucs file or a_star file
    def write_the_cost(self, cost, heuristic_cost):
        with open(RESULTIDA, 'a+') as file:
            file.write(str(cost) + ', ' + str(heuristic_cost) + '\n')

# load the map
def loadData():
  return  load_map_from_csv()


# compute travel speed between the nodeA and nodeB

def calculate(father, son):
    links = father.links
    # find the link between node A and node B
    linkAB = None
    for link in links:
        if link.target == son.index:
            linkAB= link
    # find speed range in the road type of the link
    if(linkAB==None):
        print("error")
    speed_range_road = info.SPEED_RANGES[linkAB.highway_type]
    return linkAB.distance / max(speed_range_road)


def sum_cost(cost,node,parent):
    return cost + calculate(parent,node)


# calculate the path between the node and the target
# return the cost and the path itself
def calculate_path_and_cost(node):
    path = []
    cost = 0
    parent = node.parents
    # iterative method to sum all cost
    while parent is not None:
        cost = sum_cost(cost,node, parent)
        path.append(node.index)
        node = parent  # node.parents
        parent = node.parents
    path.append(node.index)
    path.reverse()
    return (path, cost)



# implement of ucs + astar algorithrms!!
# @tools.timed
def find_best_path(source, target, roads,heuristic):
    #class of node
    node = State(roads[source])
    end_node = State(roads[target])
    open = []
    close = set()
    # change open to min heap
    heapq.heappush(open, node)
    TABLE_COST[node.index] = 0
    while len(open) != 0:
        node = heapq.heappop(open)
        # get the neighbors of node
        list_neighbors = node.get_neightbors(roads)
        close.add(node.index)
        # if we reach the target
        if node.index == end_node.index:
            list_of_path, cost = calculate_path_and_cost(node)
            heuristic_cost = heuirstic_function(State(roads[source]), end_node)
            # node.write_the_cost(cost,heuristic_cost)
            return list_of_path
            # return list_of_path, cost
        for s in list_neighbors:
            if s.index not in close:
                # calculate the cost
                new_cost = TABLE_COST[node.index] + calculate(node, s) + heuristic(s,end_node)
                # if we already been there
                if s in open:
                    old_node = open[open.index(s)]
                    if TABLE_COST[old_node.index] > new_cost:
                        # remove old state, update his state and push back to heap
                        open.remove(old_node)
                        # make the list heap
                        heapq.heapify(open)
                        TABLE_COST[old_node.node.index] = new_cost
                        old_node.parents = node
                        heapq.heappush(open, old_node)
                else:
                     s.parents = node
                     TABLE_COST[s.index] = new_cost
                     heapq.heappush(open, s)
    return None


# solve the  problem in ucs algorithm ( also was to write the result on the file)
def solve_the_problems_ucs(source, target):
    roads, lines = load_information()
    # This is for writing and testing!

    # for line in lines:
    #     source = int(line[0])
    #     target = int(line[1])
        # the lambda function is just 0 because we don't user heuricstic function,
        # it's just for generic code.
    return find_best_path(source, target, roads, lambda source,target: 0)

# General function to read the problems and load the roads.
def load_information():
    fp = open(PROBLEMS)
    reader = csv.reader(fp)
    lines = list(reader)
    roads = loadData()
    return roads,lines

# This belong to A* Algorithm!
def heuirstic_function(s,target):
    max_speed = 0
    for speed in info.SPEED_RANGES:
        if max(speed) > max_speed:
            max_speed = max(speed)
    speed_range_road = max_speed
    result = 1000 * compute_distance(s.lat,s.lon,target.lat,target.lon)  / speed_range_road
    return result

# solve the  problem in a* algorithm ( also was to write the result on the file)

def solve_the_problems_a_star(source, target):
    roads,lines = load_information()
    return find_best_path (source,target,roads,heuirstic_function)

    # This is for writing and testing!
    # result = []
    # for i in range(1):
    #     line = random.choice(lines)
    #     source = int(line[0])
    #     target = int(line[1])
    #     result.append(find_best_path(source, target, roads, heuirstic_function))
    # with open('result.txt', 'w+') as f:
    #     for item in result:
    #         f.write(item)



# use of idastart algorithm!
def dfs_f(node, target, f_limit, path, g_function_result,roads):
   global new_limit
   new_f = g_function_result + heuirstic_function(node,target)
   if new_f > f_limit:
       new_limit = min(new_limit,new_f)
       return None
   if node.index == target.index:
       if path[0] != node.index and len(path) < 2:
           path.append(target.index)
       return path ,g_function_result
   for c in node.get_neightbors(roads):
       path.append(c.index)
       cost = calculate(node, c)
       final_cost =  g_function_result + cost
       sol = dfs_f(c,target,f_limit, path,  final_cost ,roads)
       if(sol):
          return sol
       path.remove(c.index)
   return None

#@tools.timed
#implement of ida* algorithm!!
def ida_algorithm(source, target, roads):
    global new_limit
    start = State(roads[source])
    end = State(roads[target])
    new_limit = heuirstic_function(start, end)
    while(1):
        f_limit = new_limit
        new_limit = math.inf
        g_function = 0
        path = [start.index]
        sol = dfs_f(start,end,f_limit ,path,g_function,roads)
        if sol:
            #start.write_the_cost(sol[1], heuirstic_function(start,end))
            return sol[0]



def solve_the_problems_idastar(source, target):
    roads, lines = load_information()
    # This is for writing and testing!

    # top5 = lines[3:8]
    # for line in top5:
    #     source =  int(line[0])
    #     target =  int(line[1])
    return ida_algorithm(source, target, roads)
