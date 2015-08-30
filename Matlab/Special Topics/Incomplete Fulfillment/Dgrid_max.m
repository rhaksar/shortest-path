classdef Dgrid_max < handle
    %DGRID Summary of this class goes here
    %   Detailed explanation goes here
    
    % define the class properties
    properties
        g
        rhs
        U
        A
        path
        v_mat
        succ
        pred
    end
    properties (Access = private)
        km
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
        function dstar = Dgrid_max(A,start,goal,succ,pred,obs,heur)
            if nargin == 7
                dstar.A = A;
                dstar.succ = succ;
                dstar.pred = pred;
                dstar.obs = obs;
                dstar.start = start;
                dstar.goal = goal;
                dstar.km = 0;
                dstar.heur = heur;
                
                dstar.U = PQueue();
                dstar.g = zeros(2,0);
                dstar.rhs = [dstar.goal; 0];
                dstar.U.Insert(dstar.goal,CalcKey(dstar.goal,dstar));
                [dstar.Nr,dstar.Nc] = size(dstar.A);
                
%                 fprintf('[Dgrid] Created class object\n');
                
            else
                fprintf('[Dgrid] Bad initialization\n');
            end
        end
        
        % find the shortest path
        function cnt = ComputeShortestPath(dstar)
            cnt = findpath(dstar);
        end
        
        % navigate the grid
        function [total_cnt,total_dist] = NavigateGrid(dstar,dflag)
            [total_cnt,total_dist] = movepath(dstar,dflag);
        end
        
        % show the current path on the grid
        function ShowPath(dstar)
            showPath(dstar.path,dstar.A);
        end
        
        % show the current grid
        function ShowGrid(dstar)
            showPath({},dstar.A);
        end
        
        % update a vertex
        function UpdateVertex(dstar,s)
            update(dstar,s);
        end
        
        % show distance values
        function ShowValues(dstar)
            showvals(dstar);
        end
        
    end
    
    
end

function showvals(dstar)
    dstar.v_mat = inf(dstar.Nc,dstar.Nr);

    for i = 1:dstar.Nc*dstar.Nr
        [r,c] = ind2sub(size(dstar.A),i);
        g_val = getVecVal(i,dstar.g);
%         [gr,gc] = ind2sub(size(dstar.A),dstar.goal);
        if g_val ~= inf
            dstar.v_mat(dstar.Nc-c+1,r) = g_val;% + dstar.heur([r,c],[gr,gc]);
        else
            dstar.v_mat(dstar.Nc-c+1,r) = -1;
        end
    end
    % v_mat = flipud(v_mat);

    figure;
    hold on;
    title('[Dgrid] Estimate of goal distances');
    h = bar3(flipud(dstar.v_mat));
    for i = 1:length(h)
         zdata = get(h(i),'Zdata');
         set(h(i),'Cdata',zdata)
    end
    axis([0 dstar.Nr+1 0 dstar.Nc+1]);
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
        [Nbors,cost] = dstar.succ(s, dstar.A);
%         lim = inf;
        lim = 0;
        for k = 1:length(Nbors)
            g_val = getVecVal(Nbors{k},dstar.g);
            val = g_val + cost(k);
            if val > lim
                lim = val;
            end
        end
        dstar.rhs = setVecVal(s,dstar.rhs,lim);
    end

    if dstar.U.Member(s)
        dstar.U.Remove(s);
%         fprintf('Removed vertex %d\n',s);
    end

    g_val = getVecVal(s,dstar.g);
    rhs_val = getVecVal(s,dstar.rhs);
    if g_val ~= rhs_val
        key = CalcKey(s,dstar);
        dstar.U.Insert(s,key);
%         fprintf('Inserted vertex %d\n',s);
    end

end

function [cnt] = findpath(dstar)
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
        
        k_old = dstar.U.TopKey();
        u = dstar.U.Pop();
        cnt = cnt + 1;
        
%         fprintf('Extracted vertex %d\n',u);
%         pause
        
        g_u = getVecVal(u,dstar.g);
        rhs_u = getVecVal(u,dstar.rhs);
        ukey = CalcKey(u,dstar);
        
        if k_old(1) < ukey(1) || (k_old(1) == ukey(1) && k_old(2) < ukey(2))
            dstar.U.Insert(u,CalcKey(u,dstar));
%             fprintf('Inserted vertex %d\n',u);
        elseif g_u > rhs_u
            dstar.g = setVecVal(u,dstar.g,rhs_u);
            [Nbors,~] = dstar.pred(u, dstar.A);
            for k = 1:length(Nbors)
                dstar.UpdateVertex(Nbors{k});
            end
        else
            dstar.g = setVecVal(u,dstar.g,inf);
            [Nbors,~] = dstar.pred(u, dstar.A);
            Nbors{end+1} = u;
            for k = 1:length(Nbors)
                dstar.UpdateVertex(Nbors{k});
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
    
%     fprintf('[Dgrid] Finished in %d set extractions\n',cnt);
end

function [total_cnt,total_dist] = movepath(dstar,dflag)
    s_last = dstar.start;
    
    total_cnt = 0;
    total_dist = 0;
    
    if isempty(dstar.g)
        cnt = dstar.ComputeShortestPath();
        total_cnt = total_cnt + cnt;
    end
    
    dstar.path = {dstar.start};
    if dflag
        showPath({},dstar.A);
        [x(1),x(2)] = ind2sub(size(dstar.A),dstar.start);
        changeColor(x,'r');
        
    end
    
    while dstar.start ~= dstar.goal
%         disp(dstar.start);
        g_startval = getVecVal(dstar.start,dstar.g);
        if g_startval == inf
            fprintf('[Dgrid] There is no possible path\n');
            return;
        end
        
        vs = dstar.obs(dstar.start,dstar.A);
        if ~isempty(vs)
            [last_r,last_c] = ind2sub(size(dstar.A),s_last);
            [start_r,start_c] = ind2sub(size(dstar.A),dstar.start);
            
            dstar.km = dstar.km + dstar.heur([last_r,last_c],[start_r,start_c]);
            s_last = dstar.start;
            
            for k = 1:length(vs)
                [r,c] = ind2sub([dstar.Nr,dstar.Nc],vs(k));
                dstar.A(r,c) = 0;
                if dflag
                    changeColor([r,c],'k');
                end
            end
            upd_vs = [];
            for k = vs
                [Nbors,~] = dstar.pred(k, dstar.A);
                upd_vs = [upd_vs k];
                for j = 1:length(Nbors)
                    upd_vs = [upd_vs Nbors{j}];
                end
            end
            for k = upd_vs
                dstar.UpdateVertex(k);
            end
            cnt = dstar.ComputeShortestPath();
            total_cnt = total_cnt + cnt;
        end

        [Nbors,cost] = dstar.succ(dstar.start, dstar.A);
        
%         lim = inf;
        lim = 0;
        key = 1;
        for k = 1:length(Nbors)
            
            g_val = getVecVal(Nbors{k},dstar.g);
            val = g_val + cost(k);
            if val > lim
                lim = val;
                key = k;
            end
        end
        dstar.start = Nbors{key};
        dstar.path{end+1} = dstar.start;
        total_dist = total_dist + cost(key);
        
        if dflag
            [x(1),x(2)] = ind2sub(size(dstar.A),dstar.start);
            changeColor(x,'r');
        end

        
    end
    
    for k = 1:length(dstar.path)
        [r,c] = ind2sub([dstar.Nr,dstar.Nc],dstar.path{k});
        dstar.path{k} = [r,c];
    end
%     fprintf('[Dgrid] Total set extractions: %d\n',total_cnt);
%     fprintf('[Dgrid] Total path length: %0.2f\n',total_dist);    

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
    axis([-0.2,N+0.2,-0.2,M+0.2]);
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
