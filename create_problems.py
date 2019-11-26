from ways import load_map_from_csv
import random
import csv
LIMIT = random.randint(10,20)
ROUND = 100
PROBLEM = "problems.csv"
# mission 3 in part 1
def randomsearch(roads):
    # mission 3 in part 1  bfs random
    name_file = PROBLEM
    result=[]
    for i in range(ROUND):
        # put a random start
        start = roads[random.randint(0,len(roads))]
        next_node = start
        for index in range(LIMIT):
            list_neighbors= list(getattr(next_node,"links"))
            new_neightbor_index = list_neighbors[random.randint(0,len(list_neighbors) - 1)].target
            next_node = roads[new_neightbor_index]
        final = next_node
        result.append([start.index, ' ' + str(final.index)])
    write_to_file(result, name_file)

# mission 3 - write to fie

def write_to_file(result, name_file):

    # write to file
    with open(name_file, 'w+',newline='') as csvF:
        writer = csv.writer(csvF)
        writer.writerows(result)
    csvF.close()

# load the map
def loadData():
  roads = load_map_from_csv()
  randomsearch(roads)

if __name__ == '__main__':
 loadData()

