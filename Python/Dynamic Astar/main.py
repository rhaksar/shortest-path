import sys
sys.path.append("C:\Users\Jay\Dropbox\Personal\Research\shortest-path\Python\common")

import dijkstra
from maze_functions import *

(start,goal,A,p,q,r) = CreateMaze(5)

heur = lambda x: 0
heur2 = lambda x: manh(x,goal,p,q)
heur3 = lambda x: diagh(x,goal,p,q,r)
heur4 = lambda x: maxh(x,goal,p,q)

neighbors = lambda x: maze4_neighbors(x,A,heur2,p,q)
d_obj = dijkstra.dijkstra(neighbors,start,goal,heur2)

total = 0
total_dist = 0
dist,cnt,path = d_obj.solve()
total += cnt

path_final = [start]
r_start = path_final[0]
idx = 1
while r_start != goal:
    vert = d_obj.path[idx]
    
    if A[vert] == 2:
        
        A[vert] = 0
        neighbors = lambda x: maze4_neighbors( x,A,heur2,p,q )
        d_obj = dijkstra.dijkstra( neighbors,r_start,goal,heur2 )
        dist,cnt,path = d_obj.solve()
        total += cnt
        
        idx = 1
        
        if len(d_obj.path) < 2:
            print "There is no possible path" 
            break
        
    
    Nbors,costs = maze4_neighbors(d_obj.path[idx-1],A,heur,p,q)    
    r_start = d_obj.path[idx]
    
    for i,nbor in enumerate(Nbors):
        if nbor == r_start:
            total_dist += costs[i]
            continue
    
    path_final.append(r_start)
        
    idx += 1

print "Total set extractions: %d" % total
print "Total path distance: %d" % total_dist