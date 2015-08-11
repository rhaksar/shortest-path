import heapq

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        # if self.isElement(item):
        #     print "This item is already in the queue: %s" % (item,)
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]
    
    def topKey(self):
        if not self.empty():
            return self.elements[0][0]
        else:
            return float("inf"),float("inf"),float("inf")
    
    def isElement(self,node):
        if not self.empty():
            for el in range(0,len(self.elements)):
                if node == self.elements[el][1]:
                    return True
        return False
    
    def remove(self,el):
        i = -1
        for j in range(len(self.elements)):
            if self.elements[j][1] == el:
                i = j
                break
        if i == -1:
            return
        
        self.elements = self.elements[:i]+self.elements[i+1:]
        heapq.heapify(self.elements)
        return