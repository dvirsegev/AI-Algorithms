import matplotlib.pyplot as plt

from ways import load_map_from_csv
from ways.draw import plot_path

RESULTASTAR = 'results/AStarRuns.txt'
RESULTIDASTAR = 'result.txt'
# width = 0.125
# l = help.generate_lists('results/AStarRuns.txt')
# for i in range(4):
#         ind = np.arange(25 * i, 25 * (i + 1))
#         plt.bar(ind + 0.125, l[0][25*i:25 * (i + 1)], width, label='ךורעיש')
#         plt.bar(ind + 0.25, l[1][25*i:25 * (i + 1)], width, label='תמא תולע')
#         plt.legend(loc='best', fontsize=12.5)
#         plt.xticks(ind + width / 2, list(x + 1 for x in ind))
#         plt.show()

def build_real_vs_heuristic(real_result,heuristic_result):
    plt.plot(heuristic_result, real_result, 'ro')
    plt.title('Real vs Heuristic')
    plt.xlabel('heuristic_result')
    plt.ylabel('Real Cost')
    plt.show()

def build_path(lines,roads):
    for line in lines:
        line = line.strip("\n")
        line = list(line.split(","))
        line = [int(i) for i in line]
        plot_path(roads,line)

def createlist():
    #file = open(RESULTASTAR,"r")
    file = open(RESULTIDASTAR, "r")
    lines = file.readlines()
    roads = load_map_from_csv()
    # real_result = []
    # heuristic_result = []
    # for line in lines:
    #     line = line.strip("\n")
    #     line = line.split(",")
    #     real_result.append(float(line[0]))
    #     heuristic_result.append(float(line[1]))
    # build_real_vs_heuristic(real_result,heuristic_result)
    build_path(lines,roads)


if __name__ == '__main__':
    createlist()