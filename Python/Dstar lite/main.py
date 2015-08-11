import os
import sys
sys.path.append(os.path.abspath(__file__ + "/../../" + "/common"))

import dstar
from maze_functions import *

(start,goal,A,p,q,r) = CreateMaze(7)

heur = lambda x: 0
heur1 = lambda x,y: 0
heur2 = lambda x,y: manh(x,y,p,q)
heur3 = lambda x,y: diagh(x,y,p,q,r)
heur4 = lambda x,y: maxh(x,y,p,q)

succ = lambda x,A: maze4_ext_neighbors(x,A,heur,p,q)
pred = lambda x,A: maze4_ext_neighbors(x,A,heur,p,q)
obs = lambda x,A: maze4_all_observer(x,A)

dstar_obj = dstar.Dgrid(A,start,goal,succ,pred,obs,heur2)
# dstar_obj.NavigateGrid()
