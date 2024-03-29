import search

def find_ucs_rout(source, target):
    'call function to find path, and return list of indices'
    #return search.find_ucs_rout(source,target)
    return search.solve_the_problems_ucs(source,target)


def find_astar_route(source, target):
    return search.solve_the_problems_a_star(source, target)


def find_idastar_route(source, target):
    return search.solve_the_problems_idastar(source, target)
    

def dispatch(argv):
    from sys import argv
    source, target = int(argv[2]), int(argv[3])
    if argv[1] == 'ucs':
        path = find_ucs_rout(source, target)
    elif argv[1] == 'astar':
        path = find_astar_route(source, target)
    elif argv[1] == 'idastar':
        path = find_idastar_route(source, target)
    print(' '.join(str(j) for j in path))


if __name__ == '__main__':
    from sys import argv
    dispatch(argv)
