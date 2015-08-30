import numpy
import os
import scipy.stats
import sys
sys.path.append(os.path.abspath(__file__ + "/../../../" + "/common"))

import dstar_max
from maze_functions import *

T = 50
S = 10
p = 0.6

w1 = numpy.linspace(0.6,2,15)
mu = 0
sigma = 0.2

w1dist = numpy.zeros(len(w1))
for i in range(0,len(w1)):
    w1dist[i] = lognpdf(w1[i],mu,sigma)
w1dist = w1dist/w1dist.sum(axis=0)

A = numpy.ones((S+1,T+1))
A[1:S+1:1,T] = 0
start = (S,0)
goal = (0,T)

Ex = numpy.multiply(w1,w1dist)
Ex = Ex.sum(axis=0)
prices = Ex*numpy.ones(T)

xk = numpy.arange(len(w1))
rvd = scipy.stats.rv_discrete(values=(xk,w1dist))

heur = lambda x: 0
heur1 = lambda x,y: 0
heur2 = lambda x,y: manh(x,y,p,q)
heur3 = lambda x,y: diagh(x,y,p,q,r)
heur4 = lambda x,y: maxh(x,y,p,q)

succ = lambda x,A: stock_ext_succ(x,A,prices)
pred = lambda x,A: stock_ext_pred(x,A,prices)
obs = lambda x,A: stock_observer(x,A,p)
# obs = lambda x,A: []


# dstar_obj = dstar_max.Dgrid(A,start,goal,succ,pred,obs,heur1)
# cnt = dstar_obj.ComputeShortestPath()
# 
# prices = numpy.zeros(T)
# for i in range(0,T):
#     prices[i] = w1[rvd.rvs()]
# 
# succ = lambda x,A: stock_ext_succ(x,A,prices)
# pred = lambda x,A: stock_ext_pred(x,A,prices)
# dstar_obj.succ = succ
# dstar_obj.pred = pred
# 
# dstar_obj.NavigateGrid()
# show_path(dstar_obj.path,A)


NMC = 10000
total_cnt = numpy.zeros(NMC)
total_dist = numpy.zeros(NMC)

for n in range(0,NMC):
    
    if n % 1000 == 0:
        print "Iteration: %d of %d" % (n, NMC)
    
    # prices = Ex*numpy.ones(T)
    prices = numpy.zeros(T)
    for i in range(0,T):
        prices[i] = w1[rvd.rvs()]
    
    succ = lambda x,A: stock_ext_succ(x,A,prices)
    pred = lambda x,A: stock_ext_pred(x,A,prices)
    
    A = numpy.ones((S+1,T+1))
    A[1:S+1:1,T] = 0
    
    dstar_obj = dstar_max.Dgrid(A,start,goal,succ,pred,obs,heur1)
    cnt = dstar_obj.ComputeShortestPath()
    
    prices = numpy.zeros(T)
    for i in range(0,T):
        prices[i] = w1[rvd.rvs()]
        
    succ = lambda x,A: stock_ext_succ(x,A,prices)
    pred = lambda x,A: stock_ext_pred(x,A,prices)
    
    dstar_obj.succ = succ
    dstar_obj.pred = pred
    
    curr_cnt,curr_dist = dstar_obj.NavigateGrid()
    
    total_cnt[n] = curr_cnt + cnt
    total_dist[n] = curr_dist
    # print ""

print "Average number of extractions: %.3f" % numpy.average(total_cnt)
print "Average path length: %.3f" % numpy.average(total_dist)