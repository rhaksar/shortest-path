classdef Dgrid < handle
    %DGRID Summary of this class goes here
    %   Detailed explanation goes here
    
    % define the class properties
    properties
        g
        rhs
        U
        A
        path
    end
    properties (Access = private)
        last
        km
        succ
        pred
        obs
        heur
        cost
        start
        goal
        Nr
        Nc
    end
    
    % define the class methods
    methods
        
        % constructor
        function dstar = Dgrid(A,s_start,s_goal,succ,pred,obs,heur,cost)
            if nargin == 8
                dstar.A = A;
                dstar.succ = succ;
                dstar.pred = pred;
                dstar.obs = obs;
                dstar.cost = cost;
                dstar.start = s_start;
                dstar.goal = s_goal;
                dstar.km = 0;
                dstar.heur = heur;
                dstar.U = PQueue();
                dstar.g = zeros(2,0);
                dstar.rhs = [dstar.goal; 0];
                dstar.U.Insert(dstar.goal,CalcKey(dstar.goal,dstar));
                [dstar.Nr,dstar.Nc] = size(dstar.A);
            else
                fprintf('Bad initialization\n');
            end
        end
        
        % find the shortest path
        function cnt = ComputeShortestPath(dstar)
            cnt = findpath(dstar);
        end
        
        % navigate the grid
        function NavigateGrid(dstar,dflag)
            movepath(dstar,dflag);
        end
        
        function ShowPath(dstar)
            showPath(dstar.path,dstar.A);
        end
        
        % update a vertex
        function UpdateVertex(dstar,s)
            update(dstar,s);
        end
        
    end
    
    
end

function key = CalcKey(s,dstar)

    g_val = getVecVal(s,dstar.g);
    rhs_val = getVecVal(s,dstar.rhs);
    
    [start_r,start_c] = ind2sub(size(dstar.A),dstar.start);
    [sr,sc] = ind2sub(size(dstar.A),s);

    key1 = min([g_val, rhs_val]) + dstar.heur([start_r,start_c],[sr,sc]) + dstar.km;
    key2 = min([g_val, rhs_val]);

    key = [key1, key2];

end

function val = getVecVal(s,vec)

    if ~isempty(vec) && ismember(s,vec(1,:))
        idx = find(s == vec(1,:));
        val = vec(2,idx);
    else
        val = inf;
    end
end

function vec = setVecVal(s,vec,val)

    if ismember(s,vec(1,:))
        idx = find(s == vec(1,:));
        vec(2,idx) = val;
    else
        vec = [vec [s; val]];
    end

end

function update(dstar,s)
    
    if s ~= dstar.goal
        Nbors = dstar.succ(s,dstar.A);
        lim = inf;
        for k = 1:length(Nbors);
            g_val = getVecVal(Nbors(k),dstar.g);
            val = g_val + dstar.cost(s,Nbors(k),dstar.A);
            if val < lim
                lim = val;
            end
        end
        dstar.rhs = setVecVal(s,dstar.rhs,lim);
    end

    if dstar.U.Member(s)
        dstar.U.Remove(s);
    end

    g_val = getVecVal(s,dstar.g);
    rhs_val = getVecVal(s,dstar.rhs);
    if g_val ~= rhs_val
        key = CalcKey(s,dstar);
        dstar.U.Insert(s,key);
    end

end

function cnt = findpath(dstar)
    UTopKey = dstar.U.TopKey();
    startKey = CalcKey(dstar.start,dstar);
    keyflag = 0;
    if UTopKey(1) < startKey(1) || (UTopKey(1) == startKey(1) && UTopKey(2) < startKey(2))
        keyflag = 1;
    end
    
    g_startval = getVecVal(dstar.start,dstar.g);
    rhs_startval = getVecVal(dstar.start,dstar.rhs);
    valflag = 0;
    if g_startval ~= rhs_startval
        valflag = 1;
    end
    
    cnt = 0;
    while keyflag || valflag
        
        u = dstar.U.Pop();
        cnt = cnt + 1;
        
        g_u = getVecVal(u,dstar.g);
        rhs_u = getVecVal(u,dstar.rhs);
        
        k_old = dstar.U.TopKey();
        ukey = CalcKey(u,dstar);
        
        if k_old(1) < ukey(1) || (k_old(1) == ukey(1) && k_old(2) < ukey(2))
            dstar.U.Insert(u,CalcKey(u,dstar));
        elseif g_u > rhs_u
            dstar.g = setVecVal(u,dstar.g,rhs_u);
            Nbors = dstar.pred(u,dstar.A);
            for k = Nbors
                dstar.UpdateVertex(k);
            end
        else
            dstar.g = setVecVal(u,dstar.g,inf);
            Nbors = dstar.pred(u,dstar.A);
            for k = [u Nbors]
                dstar.UpdateVertex(k);
            end
        end
        
        UTopKey = dstar.U.TopKey();
        startKey = CalcKey(dstar.start,dstar);
        keyflag = 0;
        if UTopKey(1) < startKey(1) || (UTopKey(1) == startKey(1) && UTopKey(2) < startKey(2))
            keyflag = 1;
        end

        g_startval = getVecVal(dstar.start,dstar.g);
        rhs_startval = getVecVal(dstar.start,dstar.rhs);
        valflag = 0;
        if g_startval ~= rhs_startval
            valflag = 1;
        end
    end
    
    fprintf('Finished in %d set extractions.\n',cnt);
end

function movepath(dstar,dflag)
    dstar.last = dstar.start;
    total = 0;
    cnt = dstar.ComputeShortestPath();
    total = total + cnt;
    
    dstar.path = {dstar.start};
    if dflag
        showPath({},dstar.A);
        [x(1),x(2)] = ind2sub(size(dstar.A),dstar.start);
        changeColor(x,'r');
        
    end
    
    while dstar.start ~= dstar.goal
        
        g_startval = getVecVal(dstar.start,dstar.g);
        if g_startval == inf
            fprintf('There is no possible path.\n');
            return;
        end
        
        Nbors = dstar.succ(dstar.start,dstar.A);
        
        lim = inf;
        key = 0;
        for k = 1:length(Nbors)
            g_val = getVecVal(Nbors(k),dstar.g);
            val = g_val + dstar.cost(dstar.start,Nbors(k),dstar.A);
            if val < lim
                lim = val;
                key = k;
            end
        end
        dstar.start = Nbors(key);
        dstar.path{end+1} = dstar.start;
        
        if dflag
            [x(1),x(2)] = ind2sub(size(dstar.A),dstar.start);
            changeColor(x,'r');
        end
            
        vs = dstar.obs(dstar.start,dstar.A);
        if ~isempty(vs)
            [last_r,last_c] = ind2sub(size(dstar.A),dstar.last);
            [start_r,start_c] = ind2sub(size(dstar.A),dstar.start);
            dstar.km = dstar.km + dstar.heur([last_r,last_c],[start_r,start_c]);
            dstar.last = dstar.start;
            for k = 1:length(vs)
                [r,c] = ind2sub([dstar.Nr,dstar.Nc],vs(k));
                dstar.A(r,c) = 0;
            end
            for k = vs
                dstar.UpdateVertex(k);
            end
            q_vert = dstar.U.GetElements();
            for k = q_vert
                dstar.U.Update(k,CalcKey(k,dstar));
            end
            cnt = dstar.ComputeShortestPath();
            total = total + cnt;
        end
    end
    
    for k = 1:length(dstar.path)
        [r,c] = ind2sub([dstar.Nr,dstar.Nc],dstar.path{k});
        dstar.path{k} = [r,c];
    end
    fprintf('Total set extractions: %d\n',total);
    fprintf('Created final path.\n');
    

end

function showPath(path,A)
initializeGraphics(A);
global handles;
for i=1:size(path,2)
    changeColor(path{i},'r');
    pause(0.03);    
end
end

function initializeGraphics(A)
figure;
N=size(A,1);
M=size(A,2);
global handles
handles =cell(N,M);
axis([-0.2,M+0.2,-0.2,N+0.2]);
hold on
daspect([1,1,1]);
for i=1:N
    for j=1:M
        switch A(i,j)
            case 0,
                col = 'k';
            case 1,
                col = 'w';
            case 2,
                col = 'b';
            case 3,
                col = 'g';
            case 4,
                col = 'y';
        end
        handles{i,j}=rectangle('Position',...
                     [i-1,j-1,1,1],'LineWidth',1,'FaceColor',col);
    end
end
set(gca,'XTick',[],'YTick',[]);
end

function changeColor(x,col)
    global handles;
    delete(handles{x(1),x(2)});
    handles{x(1),x(2)}=rectangle('Position',...
                     [x(1)-1,x(2)-1,1,1],'LineWidth',1,'FaceColor',col);
                 drawnow;
end
