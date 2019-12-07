import matplotlib.pyplot as plt

from ways import load_map_from_csv
from ways.draw import plot_path

RESULTASTAR= 'results/result.txt'
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
    plt.plot(real_result, heuristic_result, 'ro')
    plt.title('Real vs Heuristic')
    plt.xlabel('Real Cost')
    plt.ylabel('Heuristic Cost')
    plt.show()

def build_path(lines,roads):
    for line in lines:
        line = line.strip("\n")
        line = list(line.split(","))
        line = [int(i) for i in line]
        plot_path(roads,line)

def createlist():
    file = open(RESULTASTAR,"r")
    lines = file.readlines()
    roads = load_map_from_csv()
    build_path(lines,roads)


if __name__ == '__main__':
    createlist()