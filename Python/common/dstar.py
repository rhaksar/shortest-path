import itertools
import matplotlib
import matplotlib.pyplot as plt
import numpy

import PriorityQueue

class Dgrid:
    def __init__(self,A,start,goal,succ,pred,obs,heur):
        self.A = A
        self.start = start
        self.goal = goal
        self.succ = succ
        self.pred = pred
        self.obs = obs
        self.heur = heur
        
        self.km = 0
        
        self.g = {}
        self.rhs = {}
        self.rhs[self.goal] = 0
        
        self.counter = itertools.count()
        self.U = PriorityQueue.PriorityQueue()
        self.U.put(self.goal, self.CalcKey(self.goal))
        
        self.path = []
        self.Nr,self.Nc = A.shape       
        
        print "[D*] Created class object"
        
    def CalcKey(self,vertex):
        
        g_val = self.g.get(vertex,float("inf"))
        rhs_val = self.rhs.get(vertex,float("inf"))
        
        el1 = min(g_val,rhs_val) + self.heur(self.start,vertex) + self.km
        el2 = min(g_val,rhs_val)
        
        return el1,el2
    
    def CompareKeys(self,a,b):
        (f1,g1) = a
        (f2,g2) = b
        
        res = False
        if f1 < f2:
            res = True
        elif f1 == f2 and g1 < g2:
            res = True
        
        return res
    
    def UpdateVertex(self,vertex):
        if vertex != self.goal:
            Nbors,costs = self.succ(vertex,self.A)
            lim = float("inf")
            for idx,vert in enumerate(Nbors):
                val = self.g.get(vert,float("inf")) + costs[idx]
                if val < lim:
                    lim = val
            
            self.rhs[vertex] = lim
    
        if self.U.isElement(vertex):
            self.U.remove(vertex)
        
        g_val = self.g.get(vertex,float("inf"))
        rhs_val = self.rhs.get(vertex,float("inf"))
        if g_val != rhs_val:
            self.U.put(vertex, self.CalcKey(vertex))
        
        return 

    def ComputeShortestPath(self):
        
        start_g_val = self.g.get(self.start,float("inf"))
        start_rhs_val = self.rhs.get(self.start,float("inf"))
        
        cnt = 0
        
        while self.CompareKeys(self.U.topKey(),self.CalcKey(self.start)) or start_g_val != start_rhs_val:
            
            k_old = self.U.topKey()
            u = self.U.get()
            cnt += 1
            
            u_g_val = self.g.get(u,float("inf"))
            u_rhs_val = self.rhs.get(u,float("inf"))
            
            if self.CompareKeys(k_old,self.CalcKey(u)):
                self.U.put(vertex,self.CalcKey(u))
            elif u_g_val > u_rhs_val:
                self.g[u] = u_rhs_val
                Nbors,costs = self.pred(u,self.A)
                for vertex in Nbors:
                    self.UpdateVertex(vertex)
            
            else:
                self.g[u] = float("inf")
                Nbors,costs = self.pred(u,self.A)
                Nbors.append(u)
                for vertex in Nbors:
                    self.UpdateVertex(vertex)
                    
            
            start_g_val = self.g.get(self.start,float("inf"))
            start_rhs_val = self.rhs.get(self.start,float("inf"))
            
        
        print "[D*] Found path in %d queue extraction(s)" % cnt
        return cnt
        
    def NavigateGrid(self):
        s_last = self.start
        
        total_cnt = 0
        total_dist = 0
        cnt = self.ComputeShortestPath()
        dist = self.g.get(self.start,float("inf"))
        total_dist += dist
        total_cnt += cnt
        
        self.path.append(self.start)
        
        while self.start != self.goal:
            g_startval = self.g.get(self.start,float("inf"))
            if g_startval == float("inf"):
                print "[D*] There is no possible path"
                return
            
            Nodes = self.obs(self.start,self.A)
            
            if Nodes:
                self.km = self.km + self.heur(s_last,self.start)
                s_last = self.start
                                  
                for vert in Nodes:
                    self.A[vert] = 0
                    
                    upd_list,costs = self.succ(vert,self.A)
                    upd_list.append(vert)
                    
                    for el in upd_list:
                        self.UpdateVertex(el)
                
                cnt = self.ComputeShortestPath()
                dist = self.g.get(self.start,float("inf"))
                total_dist += dist
                total_cnt += cnt
            
            Nbors,costs = self.succ(self.start,self.A)
            
            lim = float("inf")
            key = -1
            for idx in range(0,len(Nbors)):
                val = self.g.get(Nbors[idx],float("inf")) + costs[idx]
                if val < lim:
                    lim = val
                    key = idx
                
            self.start = Nbors[key]
            self.path.append(self.start)
        
        print "[D*] Total queue extractions: %d" % total_cnt
        print "[D*] Total path length: %d" % total_dist
            
            