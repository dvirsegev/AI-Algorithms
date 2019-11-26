'''
This file should be runnable to print map_statistics using 
$ python stats.py
'''

from collections import namedtuple, Counter
from ways import load_map_from_csv



def map_statistics(roads):
    '''return a dictionary containing the desired information
    You can edit this function as you wish'''
    Stat = namedtuple('Stat', ['max', 'min', 'avg'])
    lengths = list(len(getattr(junction,"links")) for junction in roads.junctions())
    lenght_dis = list(getattr(link,"distance") for link in roads.iterlinks())
    return {
        'Number of junctions' :len(roads),
        'Number of links' : len(list(roads.iterlinks())),
        'Outgoing branching factor' : Stat(max=max(lengths), min=min(lengths), avg=sum(lengths) / len(lengths)),
        'Link distance' : Stat(max=max(lenght_dis), min=min(lenght_dis), avg=sum(lenght_dis) / len(lenght_dis)),
        # value should be a dictionary
        # mapping each road_info.TYPE to the no' of links of this type
        'Link type histogram' : Counter(getattr(way,"highway_type") for way in roads.iterlinks()),  # tip: use collections.Counter
    }


def print_stats():
    for k, v in map_statistics(load_map_from_csv()).items():
        print('{}: {}'.format(k, v))
        
if __name__ == '__main__':
    from sys import argv
    assert len(argv) == 1
    print_stats()
