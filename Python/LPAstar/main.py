import os
import sys
sys.path.append(os.path.abspath(__file__ + "/../../" + "/common"))

import lpa
from maze_functions import *

(start,goal,A,p,q,r) = CreateMaze(4)

heur = lambda x: 0
heur2 = lambda x: manh(x,goal,p,q)
heur3 = lambda x: diagh(x,goal,p,q,r)
heur4 = lambda x: maxh(x,goal,p,q)

succ = lambda x,A: maze4_ext_neighbors(x,A,heur,p,q)
pred = lambda x,A: maze4_ext_neighbors(x,A,heur,p,q)

lpa_obj = lpa.LPAgrid(A,start,goal,succ,pred,heur2)
lpa_obj.ComputeShortestPath()

# succ = lambda x,A: maze8_ext_neighbors(x,A,heur,p,q,r)
# pred = lambda x,A: maze8_ext_neighbors(x,A,heur,p,q,r)
# 
# lpa_obj = lpa.LPAgrid(A,start,goal,succ,pred,heur4)
# lpa_obj.ComputeShortestPath()

# lpa_obj.UpdateGrid()    
# lpa_obj.ComputeShortestPath()