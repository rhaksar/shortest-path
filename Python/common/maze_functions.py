import inspect
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib
import numpy
import os
import scipy.io
import sys

def manh(x,y,p,q):
    (r1,c1) = x
    (r2,c2) = y
    c = p*abs(r1-r2) + q*abs(c1-c2)
    return c

def diagh(x,y,p,q,r):
    (r1,c1) = x
    (r2,c2) = y
    dr = abs(r2 - r1)
    dc = abs(c2 - c1)
    
    D2 = 1
    D = 1
    c = D*(dr + dc) + (D2 - 2*D)*min(dr,dc)
    return c

def maxh(x,y,p,q):
    (r1,c1) = x
    (r2,c2) = y
    dr = abs(r2 - r1)
    dc = abs(c2 - c1)
    
    return max(dr, dc)

def maze4_neighbors(x,A,h,pc,qc):
    (Nr,Nc) = A.shape
    (r,c) = x
    
    Nbors = []
    costs = []
    
    if r-1 >= 0 and A[(r-1,c)]:
        Nbors.append( (r-1,c) )
        costs.append( pc + h((r-1,c)) - h((r,c)) )
    if c+1 < Nc and A[(r,c+1)]:
        Nbors.append( (r,c+1) )
        costs.append( qc + h((r,c+1)) - h((r,c)) )
    if r+1 < Nr and A[(r+1,c)]:
        Nbors.append( (r+1,c) )
        costs.append( pc + h((r+1,c)) - h((r,c)) )
    if c-1 >= 0 and A[(r,c-1)]:
        Nbors.append( (r,c-1) )
        costs.append( qc + h((r,c-1)) - h((r,c)) )

    if (r + c) % 2 == 0:
        Nbors.reverse()
        costs.reverse()
    
    return Nbors,costs

def maze4_ext_neighbors(x,A,h,pc,qc):
    (Nr,Nc) = A.shape
    (r,c) = x
    
    Nbors = []
    costs = []
    
    if r-1 >= 0 and A[(r-1,c)]:
        Nbors.append( (r-1,c) )
        if A[r,c]:
            costs.append( pc + h((r-1,c)) - h((r,c)) )
        else:
            costs.append(float("inf"))
            
    if c+1 < Nc and A[(r,c+1)]:
        Nbors.append( (r,c+1) )
        if A[r,c]:
            costs.append( qc + h((r,c+1)) - h((r,c)) )
        else:
            costs.append(float("inf"))
            
    if r+1 < Nr and A[(r+1,c)]:
        Nbors.append( (r+1,c) )
        if A[r,c]:
            costs.append( pc + h((r+1,c)) - h((r,c)) )
        else:
            costs.append(float("inf"))
            
    if c-1 >= 0 and A[(r,c-1)]:
        Nbors.append( (r,c-1) )
        if A[r,c]:
            costs.append( qc + h((r,c-1)) - h((r,c)) )
        else:
            costs.append(float("inf"))
    
    if (r + c) % 2 == 0:
        Nbors.reverse()
        costs.reverse()
    
    return Nbors,costs

def maze8_neighbors(x,A,h,pc,qc,rc):
    (Nr,Nc) = A.shape
    (r,c) = x
    
    Nbors = []
    costs = []
    
    if r-1 >= 0 and A[(r-1,c)]:
        Nbors.append((r-1,c))
        costs.append( pc + h((r-1,c)) - h((r,c)) )
    if c+1 < Nc and A[(r,c+1)]:
        Nbors.append((r,c+1))
        costs.append( qc + h((r,c+1)) - h((r,c)) )
    if r+1 < Nr and A[(r+1,c)]:
        Nbors.append((r+1,c))
        costs.append( pc + h((r+1,c)) - h((r,c)) )
    if c-1 >= 0 and A[(r,c-1)]:
        Nbors.append((r,c-1))
        costs.append( qc + h((r,c-1)) - h((r,c)) )
        
    if r-1 >= 0 and c-1 >= 0 and A[(r-1,c-1)]:
        Nbors.append((r-1,c-1))
        costs.append( rc + h((r-1,c-1)) - h((r,c)) )
    if r+1 < Nr and c+1 < Nc and A[(r+1,c+1)]:
        Nbors.append( (r+1,c+1) )
        costs.append( rc + h((r+1,c+1)) - h((r,c)) )
    if r-1 >= 0 and c+1 < Nc and A[(r-1,c+1)]:
        Nbors.append( (r-1,c+1) )
        costs.append( rc + h((r-1,c+1)) - h((r,c)) )
    if r+1 < Nr and c-1 >= 0 and A[(r+1,c-1)]:
        Nbors.append( (r+1,c-1) )
        costs.append( rc + h((r+1,c-1)) - h((r,c)) )

    if (r + c) % 2 == 0:
        Nbors.reverse()
        costs.reverse()
    
    return Nbors,costs

def maze8_ext_neighbors(x,A,h,pc,qc,rc):
    (Nr,Nc) = A.shape
    (r,c) = x
    
    Nbors = []
    costs = []
    
    if r-1 >= 0 and A[(r-1,c)]:
        Nbors.append( (r-1,c) )
        if A[r,c]:
            costs.append( pc + h((r-1,c)) - h((r,c)) )
        else:
            costs.append(float("inf"))
            
    if c+1 < Nc and A[(r,c+1)]:
        Nbors.append( (r,c+1) )
        if A[r,c]:
            costs.append( qc + h((r,c+1)) - h((r,c)) )
        else:
            costs.append(float("inf"))
            
    if r+1 < Nr and A[(r+1,c)]:
        Nbors.append( (r+1,c) )
        if A[r,c]:
            costs.append( pc + h((r+1,c)) - h((r,c)) )
        else:
            costs.append(float("inf"))
            
    if c-1 >= 0 and A[(r,c-1)]:
        Nbors.append( (r,c-1) )
        if A[r,c]:
            costs.append( qc + h((r,c-1)) - h((r,c)) )
        else:
            costs.append(float("inf"))
    
    if r-1 >= 0 and c-1 >= 0 and A[(r-1,c-1)]:
        Nbors.append((r-1,c-1))
        if A[r,c]:
            costs.append( rc + h((r-1,c-1)) - h((r,c)) )
        else:
            costs.append(float("inf"))

    if r+1 < Nr and c+1 < Nc and A[(r+1,c+1)]:
        Nbors.append( (r+1,c+1) )
        if A[r,c]:
            costs.append( rc + h((r+1,c+1)) - h((r,c)) )
        else:
            costs.append(float("inf"))

    if r-1 >= 0 and c+1 < Nc and A[(r-1,c+1)]:
        Nbors.append( (r-1,c+1) )
        if A[r,c]:
            costs.append( rc + h((r-1,c+1)) - h((r,c)) )
        else:
            costs.append(float("inf"))
        
    if r+1 < Nr and c-1 >= 0 and A[(r+1,c-1)]:
        Nbors.append( (r+1,c-1) )
        if A[r,c]:
            costs.append( rc + h((r+1,c-1)) - h((r,c)) )
        else:
            costs.append(float("inf"))
            
    if (r + c) % 2 == 0:
        Nbors.reverse()
        costs.reverse()
    
    return Nbors,costs

def maze4_all_observer( x,A ):
    Nr,Nc = A.shape
    r,c = x
    
    Nodes = []
    t = 2
    
    if r+1 < Nr and A[(r+1,c)] >= t:
        Nodes.append((r+1,c))
    
    if r-1 >= 0 and A[(r-1,c)] >= t:
        Nodes.append((r-1,c))
        
    if c+1 < Nc and A[(r,c+1)] >= t:
        Nodes.append((r,c+1))
    
    if c-1 >= 0 and A[(r,c-1)] >= t:
        Nodes.append((r,c-1))
        
    return Nodes


def CreateMaze(c):
    if c == 1:
        A = numpy.ones( (5,5) )
        A[4,0] = 0
        # A[1,1] = 0
        start = (0,0)
        goal = (2,2)
        p = 1
        q = 1
        r = 0
        
    elif c == 2:
        A = numpy.ones( (81,81) )
        start = (62,0)
        goal = (20,80)
        p = 1
        q = 1
        r = 0
        
    elif c == 3:
        file_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + '\LPA_maze8_2.mat'
        mat = scipy.io.loadmat(file_path)
        
        A = mat['A']        
        start = (7,4)
        goal = (7,16)
        p = 1
        q = 1
        r = 1
    
    elif c == 4:
        file_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + '\EE266_maze4.mat'
        mat = scipy.io.loadmat(file_path)
        
        A = mat['A']
        start = (62,0)
        goal = (20,80)
        p = 2
        q = 4
        r = 0
    
    elif c == 5:
        file_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + '\EE266_maze4.mat'
        mat = scipy.io.loadmat(file_path)
        
        A = mat['A']
        A[45,54] = 2
        
        start = (62,0)
        goal = (20,80)
        p = 2
        q = 4
        r = 0
        
    elif c == 6:
        file_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + '\LPA_maze8_1.mat'
        mat = scipy.io.loadmat(file_path)
        
        A = mat['A']
        A[3,1] = 2
        start = (0,3)
        goal = (5,0)
        p = 1
        q = 1
        r = 1
    
    elif c == 7:
        file_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + '\EE266_maze4.mat'
        mat = scipy.io.loadmat(file_path)
        
        A = mat['A']
        
        Nr,Nc = A.shape
        for i in range(0,Nr):
            for j in range(0,Nc):
                if A[(i,j)] == 0:
                    A[(i,j)] = 2
        
        start = (62,0)
        goal = (20,80)
        p = 2
        q = 4
        r = float("inf")
        
    elif c == 8:
        file_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + '\LPA_maze8_2.mat'
        mat = scipy.io.loadmat(file_path)
        
        A = mat['A']
        A[(7,12)] = 0
        
        Nr,Nc = A.shape
        for i in range(0,Nr):
            for j in range(0,Nc):
                if A[(i,j)] == 0:
                    A[(i,j)] = 2
        
        start = (7,3)
        goal = (7,16)
        p = 1
        q = 1
        r = 1
    
    return start,goal,A,p,q,r

def ind2sub( shape,idx ):
    nrows = shape[0]
    r = idx % nrows 
    c = (idx-r) / nrows 
    return r,c

def sub2ind( shape,r,c ):
    return r + c*shape[0]

def show_values(values,A):
    fig = plt.figure()
    ax = Axes3D(fig)
    
    (lx,ly) = A.shape
    data = numpy.zeros((lx,ly))
    
    for value,key in enumerate(values):
        data[key] = values[key]
    
    # print "%s" % data
    # print("%s" %(data))
        
    xpos = numpy.arange(0,lx,1)
    ypos = numpy.arange(0,ly,1)
    xpos, ypos = numpy.meshgrid(xpos+0.5, ypos+0.5)
    
    xpos = xpos.flatten()
    ypos = ypos.flatten()
    zpos = numpy.zeros(lx*ly)
    
    dx = numpy.ones_like(zpos)
    dy = dx.copy()
    dz = data.flatten(order = 'F')
    
    # print("%s" % (dz))
    fracs = dz.astype(float)/dz.max()
    norm = matplotlib.colors.Normalize(fracs.min(), fracs.max())
    colors = matplotlib.cm.gist_rainbow(norm(fracs))
    
    ax.bar3d(xpos,ypos,zpos, dx,dy,dz, color=colors)
    
    # ax.bar3d(xpos,ypos,zpos, dx,dy,dz, color = 'b')
    plt.show()
    
    return
    
def show_path(path,A):
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    
    (Nr,Nc) = A.shape
    for r in range(Nr):
        for c in range(Nc):
            
            vert = (r,c)
            if vert in path:
                A[r,c] = 5
            
            if A[r,c] == 0:
                col = 'black'
                fl = True
            elif A[r,c] == 1:
                col = 'black'
                fl = False
            elif A[r,c] == 2:
                col = 'blue'
                fl = True
            elif A[r,c] == 3:
                col = 'green'
                fl = True
            elif A[r,c] == 4:
                col = 'yellow'
                fl = True
            elif A[r,c] == 5:
                col = 'red'
                fl = True
                
            ax1.add_patch(matplotlib.patches.Rectangle( (c-0.5,Nr-r-0.5),1,1,color=col,fill=fl,linewidth=1))
            # fig1.canvas.draw()
    
    # ax1.add_patch(matplotlib.patches.Rectangle( (0.1,0.1),0.5,0.5, ))
    # plt.pcolor(numpy.random.random((10,10)),cmap='gist_rainbow')
    # plt.colorbar()
    plt.xlim([-1, Nc])
    plt.ylim([0, Nr + 1])
    
    # for idx,vert in enumerate(path):
    #     (r,c) = vert
    #     ax1.annotate(idx,xy=(c,Nr-r))
    #     ax1.add_patch(matplotlib.patches.Rectangle( (c-0.5,Nr-r-0.5),1,1,color='red',fill=True))
    #     fig1.canvas.draw()
    
    plt.show()
    return
