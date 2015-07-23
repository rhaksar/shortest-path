classdef LPAgrid < handle
    %LPAGRID Summary of this class goes here
    %   Detailed explanation goes here
    
    % define the class properties
    properties
        g
        rhs
        U
        path
        A
    end
    properties (Access = private)
        succ
        pred
        heur
        cost
        start
        goal
        path_plot
        Nr
        Nc
    end
    
    % define the class methods
    methods
        % constructor
        function lpa = LPAgrid(A,s_start,s_goal,succ,pred,heur,cost)
            if nargin == 7
                lpa.A = A;
                lpa.succ = succ;
                lpa.pred = pred;
                lpa.cost = cost;
                lpa.start = s_start;
                lpa.goal = s_goal;
                lpa.heur = heur;
                lpa.U = PQueue();
                lpa.g = zeros(2,0);
                lpa.rhs = [lpa.start; 0];
                lpa.U.Insert(lpa.start,CalcKey(lpa.start,lpa));
                lpa.path = {};
                lpa.path_plot = {};
                [lpa.Nr,lpa.Nc] = size(lpa.A);
            end
        end
        
        % compute the shortest path
        function ComputeShortestPath(lpa)
            findpath(lpa);  
        end
        
        % update vertices
        function UpdateVertex(lpa,s)
    
            if s ~= lpa.start
                Nbors = lpa.pred(s,lpa.A);
                lim = inf;
                for k = 1:length(Nbors);
                    g_val = getVecVal(Nbors(k),lpa.g);
                    val = g_val + lpa.cost(Nbors(k),s,lpa.A);
                    if val < lim
                        lim = val;
                    end
                end
                lpa.rhs = setVecVal(s,lpa.rhs,lim);
            end

            if lpa.U.Member(s)
                lpa.U.Remove(s);
            end

            g_val = getVecVal(s,lpa.g);
            rhs_val = getVecVal(s,lpa.rhs);
            if g_val ~= rhs_val
                key = CalcKey(s,lpa);
                lpa.U.Insert(s,key);
            end

        end
        
        % create path
        function CreatePath(lpa)
            makepath(lpa);
        end
        
        % show path
        function ShowPath(lpa)
            showPath(lpa.path_plot,fliplr(lpa.A'));
        end
        
        % show grid only
        function ShowGrid(lpa)
            showPath({},fliplr(lpa.A'));
        end
        
        
    end
    
end

function key = CalcKey(s,lpa)

    g_val = getVecVal(s,lpa.g);
    rhs_val = getVecVal(s,lpa.rhs);

    key1 = min([g_val, rhs_val]) + lpa.heur(s,lpa.goal);
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

function makepath(lpa)
    g_goal = getVecVal(lpa.goal,lpa.g);
    if g_goal == inf
        fprintf('No such path can be created.\n');
        lpa.path = {};
        lpa.path_plot = {};
        return;
    end

    lpa.path = {lpa.goal};

    vertex = lpa.goal;
    while vertex ~= lpa.start
        Nbors = lpa.succ(vertex,lpa.A);

        lim = inf;
        key = 0;
        for k = 1:length(Nbors)
            g_val = getVecVal(Nbors(k),lpa.g);
            val = g_val + lpa.cost(vertex,Nbors(k),lpa.A);
            if val < lim
                lim = val;
                key = k;
            end
        end

        vertex = Nbors(key);
        lpa.path{end+1} = vertex;
    end

    lpa.path = lpa.path(end:-1:1);

    for i = 1:length(lpa.path)
        [r,c] = ind2sub([lpa.Nr,lpa.Nc],lpa.path{i});
        lpa.path{i} = [r,c];
        lpa.path_plot{i} = [c,lpa.Nr-r+1];
    end
    
    fprintf('Created optimal path.\n');
end

function lpa = findpath(lpa)
    UTopKey = lpa.U.TopKey();
    goalKey = CalcKey(lpa.goal,lpa);
    keyflag = 0;
    if UTopKey(1) < goalKey(1) || (UTopKey(1) == goalKey(1) && UTopKey(2) < goalKey(2))
        keyflag = 1;
    end
    
    g_goalval = getVecVal(lpa.goal,lpa.g);
    rhs_goalval = getVecVal(lpa.goal,lpa.rhs);
    valflag = 0;
    if g_goalval ~= rhs_goalval
        valflag = 1;
    end

    cnt = 0;
    while keyflag || valflag
        
        u = lpa.U.Pop();
        cnt = cnt + 1;
        
        g_u = getVecVal(u,lpa.g);
        rhs_u = getVecVal(u,lpa.rhs);
        
        if g_u > rhs_u
            lpa.g = setVecVal(u,lpa.g,rhs_u);
            Nbors = lpa.succ(u,lpa.A);
            for k = Nbors
                lpa.UpdateVertex(k);
            end
        else
            lpa.g = setVecVal(u,lpa.g,inf);
            Nbors = lpa.succ(u,lpa.A);
            for k = [u Nbors]
                lpa.UpdateVertex(k);
            end
        end
        
        UTopKey = lpa.U.TopKey();
        goalKey = CalcKey(lpa.goal,lpa);
        if UTopKey(1) < goalKey(1) || (UTopKey(1) == goalKey(1) && UTopKey(2) < goalKey(2))
            keyflag = 1;
        else
            keyflag = 0;
        end

        g_goalval = getVecVal(lpa.goal,lpa.g);
        rhs_goalval = getVecVal(lpa.goal,lpa.rhs);
        valflag = 0;
        if g_goalval ~= rhs_goalval
            valflag = 1;
        end
        
    end
    
    fprintf('Finished in %d set extractions\n',cnt);
    
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
% axis([-0.2,M+0.2,-0.2,N+0.2]);
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
end
