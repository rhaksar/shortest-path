import os
import sys
sys.path.append(os.path.abspath(__file__ + "/../../" + "/common"))

import dijkstra
from maze_functions import *

(start,goal,A,p,q,r) = CreateMaze(1)

heur = lambda x: 0
heur2 = lambda x: manh(x,goal,p,q)
heur3 = lambda x: diagh(x,goal,p,q,r)
heur4 = lambda x: maxh(x,goal,p,q)

# neighbors = lambda x: maze4_neighbors(x,A,heur,p,q)
# d_obj = dijkstra.dijkstra(neighbors,start,goal,heur)
# dist,cnt,path = d_obj.solve()
# show_path(d_obj.path,A)
# show_values(d_obj.cost_so_far,A)
# print "\n"

neighbors = lambda x: maze4_neighbors(x,A,heur2,p,q)
d_obj = dijkstra.dijkstra(neighbors,start,goal,heur2)
dist,cnt,path = d_obj.solve()
# show_path(d_obj.path,A)
show_values(d_obj.cost_so_far,A)

# neighbors = lambda x: maze8_neighbors( x,A,heur3,p,q,r )
# d_obj = dijkstra.dijkstra( neighbors,start,goal,heur3 )
# dist,cnt,path = d_obj.solve()
# show_path(d_obj.path, A)
# show_values(d_obj.cost_so_far, A)

print "Total path length: %d" % dist