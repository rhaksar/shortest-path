import itertools
import matplotlib
import matplotlib.pyplot as plt
import numpy

import PriorityQueue 

class dijkstra:
    def __init__(self, neighbors,start,goal,heur ):
        self.neighbors = neighbors
        self.start = start
        self.goal = goal
        self.came_from = {}
        self.cost_so_far = {}
        self.path = []
        self.heur = heur
        
    def solve(self):
        frontier = PriorityQueue.PriorityQueue()
        counter = itertools.count()
        frontier.put(self.start, (0,-next(counter)))
        # self.came_from = {}
        # selfcost_so_far = {}
        self.came_from[self.start] = None
        self.cost_so_far[self.start] = 0
        cnt = 0;
        
        while not frontier.empty():
            # print("%s" % frontier.elements)
            current = frontier.get()
            cnt = cnt + 1;
            # print("%s" % (current,))
            
            if current == self.goal:
                break
            
            (Nbors,costs) = self.neighbors(current)
            for idx, nex in enumerate(Nbors):
                new_cost = self.cost_so_far[current] + costs[idx]
                if nex not in self.cost_so_far or new_cost < self.cost_so_far[nex]:
                    self.cost_so_far[nex] = new_cost
                    priority = new_cost
                    frontier.put(nex, (priority,-next(counter)))
                    self.came_from[nex] = current
        
        for key,value in self.cost_so_far.items():
            self.cost_so_far[key] = value + self.heur(self.start)
        
        dist = self.cost_so_far[self.goal]
        print "Finished in %d queue extractions" % cnt
        
        self.make_path()
        
        return dist, cnt, self.path

    def make_path(self):
        current = self.goal
        self.path = [current]
        while current != self.start:
            current = self.came_from[current]
            self.path.append(current)
        self.path.reverse()
        return
    