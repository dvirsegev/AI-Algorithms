import matplotlib.pyplot as plt
import numpy as np
RESULTASTAR= 'results/AStarRuns.txt'
# width = 0.125
# l = help.generate_lists('results/AStarRuns.txt')
# for i in range(4):
#         ind = np.arange(25 * i, 25 * (i + 1))
#         plt.bar(ind + 0.125, l[0][25*i:25 * (i + 1)], width, label='ךורעיש')
#         plt.bar(ind + 0.25, l[1][25*i:25 * (i + 1)], width, label='תמא תולע')
#         plt.legend(loc='best', fontsize=12.5)
#         plt.xticks(ind + width / 2, list(x + 1 for x in ind))
#         plt.show()

def build_graph(real_result,heuristic_result):
    width = 0.5
    for i in range(4):
        ind = np.arange(25 * i, 25 * (i + 1))
        plt.bar(ind + 0.125, heuristic_result[25*i:25 * (i + 1)], width, label='ךורעיש')
        plt.bar(ind + 0.25, real_result[25*i:25 * (i + 1)], width, label='תמא תולע')
        plt.legend(loc='best', fontsize=12.5)
        plt.xticks(ind + width / 2, list(x + 1 for x in ind))
        plt.show()

def createlist():
    real_result = []
    heuristic_result = []
    file = open(RESULTASTAR,"r")
    lines = file.read().splitlines()
    for line in lines:
        x = line.split(',')
        real_result.append(x[0])
        heuristic_result.append(x[1])
    build_graph(real_result,heuristic_result)


if __name__ == '__main__':
    createlist()