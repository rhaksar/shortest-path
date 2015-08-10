import itertools
import matplotlib
import matplotlib.pyplot as plt
import numpy

import PriorityQueue

class LPAgrid:
    def __init__(self,A,start,goal,succ,pred,heur):
        self.A = A
        self.succ = succ
        self.pred = pred
        self.start = start
        self.goal = goal
        self.heur = heur
        
        self.g = {}    
        self.rhs = {}
        self.rhs[self.start] = 0 
        
        self.counter = itertools.count()
        self.U = PriorityQueue.PriorityQueue()
        self.U.put(self.start, self.CalcKey(self.start)+(-next(self.counter),))

        self.path = []
        self.Nr,self.Nc = A.shape
        print "[LPA*] Created class object"
        
    def CalcKey(self,vertex):
        
        g_val = self.g.get(vertex,float("inf"))
        rhs_val = self.rhs.get(vertex,float("inf"))
        
        el1 = min(g_val,rhs_val) + self.heur(vertex)
        el2 = min(g_val,rhs_val)
        
        return el1,el2
    
    def CompareKeys(self,a,b):
        (f1,g1,a1) = a
        (f2,g2,a2) = b
        
        res = False
        if f1 < f2:
            res = True
        elif f1 == f2 and g1 < g2:
            res = True
        # elif f1 == f2 and g1 == g2 and a1 < a2:
        #     res = True
        
        return res
    
    def UpdateVertex(self,vertex):
        if vertex != self.start:
            Nbors,costs = self.pred(vertex,self.A)
            lim = float("inf")
            for idx,verts in enumerate(Nbors):
                val = self.g.get(verts,float("inf")) + costs[idx]
                if val < lim:
                    lim = val
            
            self.rhs[vertex] =  lim
        
        if self.U.isElement(vertex):
            self.U.remove(vertex)
            # print "Removed vertex %s" % (vertex,)
        
        g_val = self.g.get(vertex,float("inf"))
        rhs_val = self.rhs.get(vertex,float("inf"))
        if g_val != rhs_val:
            self.U.put(vertex, self.CalcKey(vertex)+(-next(self.counter),))
            # print "Inserted vertex %s" % (vertex,)
    
    def ComputeShortestPath(self):
        
        goal_g_val = self.g.get(self.goal,float("inf"))
        goal_rhs_val = self.rhs.get(self.goal,float("inf"))
        goalKey = self.CalcKey(self.goal) + (float("inf"),)
        topKey = self.U.topKey()
        
        cnt = 0
        
        while self.CompareKeys(topKey,goalKey) or goal_g_val != goal_rhs_val:
            
            # print "%s" % self.U.elements
            
            u = self.U.get()
            cnt += 1
            
            # print "%s" % (u,) 
            
            g_val = self.g.get(u,float("inf"))
            rhs_val = self.rhs.get(u,float("inf"))
            
            if g_val > rhs_val:
                self.g[u] = self.rhs[u]
                (Nbors,costs) = self.succ(u,self.A)
                for vertex in Nbors:
                    self.UpdateVertex(vertex)
            else:
                self.g[u] = float("inf")
                (Nbors,costs) = self.succ(u,self.A)
                Nbors.append(u)
                for vertex in Nbors:
                    self.UpdateVertex(vertex)
            
            goal_g_val = self.g.get(self.goal,float("inf"))
            goal_rhs_val = self.rhs.get(self.goal,float("inf"))
            goalKey = self.CalcKey(self.goal) + (float("inf"),)
            topKey = self.U.topKey()
            
            # print "%d" % cnt
            # print "\n"
        
        print "[LPA*] Finished in %d queue extractions" % cnt
        self.make_path()
        
        return cnt
    
    def make_path(self):
        if self.g.get(self.goal,float("inf")) == float("inf"):
            print "[LPA*] No possible path"
            return
        
        self.path = []
        self.path.append(self.goal)
        vertex = self.goal
        
        while vertex != self.start:
            Nbors,costs = self.succ(vertex,self.A)
            
            lim = float("inf")
            key = -1
            for idx in range(0,len(Nbors)):
                val = self.g.get(Nbors[idx],float("inf")) + costs[idx]
                if val < lim:
                    lim = val
                    key = idx
                    
            vertex = Nbors[key]
            self.path.append(vertex)
            
        self.path.reverse()   
        print "[LPA*] Created optimal path of length %d" % self.g.get(self.goal)
        
        return
    
    def UpdateGrid(self):
        
        row,col = (self.A == 2).nonzero()
        
        if row.size == 0:
            print "[LPA*] No new information to update grid"
            return
        
        self.A[row,col] = 0
        
        for idx,val in enumerate(row):
            upd_list = []
            
            vert = (row[idx],col[idx])
            Nbors,costs = self.succ(vert,self.A)
            
            upd_list.append(vert)
            for vertex in Nbors:
                upd_list.append(vertex)
        
            for vertex in upd_list:
                self.UpdateVertex(vertex)
        
        print "[LPA*] Updated grid with new information"
        
        return