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
        start
        v_mat
    end
    properties (Access = private)
        succ
        pred
        heur
        cost
        goal
        path_plot
        Nr
        Nc
    end
    
    % define the class methods
    methods
        % constructor
        function lpa = LPAgrid(A,start,goal,succ,pred,heur)
            if nargin == 6
                lpa.A = A;
                lpa.succ = succ;
                lpa.pred = pred;
                lpa.start = start;
                lpa.goal = goal;
                lpa.heur = heur;
                
                lpa.U = PQueue();
                lpa.g = zeros(2,0);
                lpa.rhs = [lpa.start; 0];
                lpa.U.Insert(lpa.start,CalcKey(lpa.start,lpa));
                
                lpa.path = {};
                lpa.path_plot = {};
                [lpa.Nr,lpa.Nc] = size(lpa.A);
                fprintf('[LPAgrid] Created class object\n');
                
            else
                fprintf('[LPAgrid] Bad initialization\n');
            end
        end
        
        % compute the shortest path
        function ComputeShortestPath(lpa)
            findpath(lpa);  
        end
        
        % update vertices
        function UpdateVertex(lpa,s)
    
            if s ~= lpa.start
                [Nbors,costs] = lpa.pred(s,lpa.A);
                lim = inf;
                for k = 1:length(Nbors);
                    g_val = getVecVal(Nbors{k},lpa.g);
                    val = g_val + costs(k);
%                     val = g_val + lpa.cost(Nbors(k),s,lpa.A);
                    if val < lim
                        lim = val;
                    end
                end
                lpa.rhs = setVecVal(s,lpa.rhs,lim);
            end

            if lpa.U.Member(s)
%                 fprintf('Removed vertex %d\n', s);
                lpa.U.Remove(s);
            end

            g_val = getVecVal(s,lpa.g);
            rhs_val = getVecVal(s,lpa.rhs);
            if g_val ~= rhs_val
                key = CalcKey(s,lpa);
                lpa.U.Insert(s,key);
%                 fprintf('Inserted vertex %d\n', s);
            end

        end
        
        % update grid with new information
        function UpdateGrid(lpa)
            vs = find(lpa.A == 2);
            
            if isempty(vs)
                fprintf('[LPAgrid] No new information\n');
                return;
            end
            
            for k = 1:length(vs)
                [r,c] = ind2sub(size(lpa.A),vs(k));
                lpa.A(r,c) = 0; 
                [Nbors,~] = lpa.succ(vs(k),lpa.A);
                Nbors{end+1} = vs(k);
                
                for j = 1:length(Nbors)
                    lpa.UpdateVertex(Nbors{k});
                end
  
            end
            
            fprintf('[LPAgrid] Updated grid with new information\n');

        end
        
        % create path
        function CreatePath(lpa)
            makepath(lpa);
        end
        
        % show path
        function ShowPath(lpa)
            showPath(lpa.path,lpa.A);
        end
        
        % show grid only
        function ShowGrid(lpa)
            showPath({},fliplr(lpa.A'));
        end
        
        % show g values on grid
        function ShowValues(lpa)
            showvals(lpa);
        end
        
    end
    
end

function showvals(lpa)
    lpa.v_mat = inf(lpa.Nc,lpa.Nr);

    for i = 1:lpa.Nc*lpa.Nr
        [r,c] = ind2sub(size(lpa.A),i);
        g_val = getVecVal(i,lpa.g);
        if g_val ~= inf
            lpa.v_mat(lpa.Nc-c+1,r) = g_val;
        else
            lpa.v_mat(lpa.Nc-c+1,r) = NaN;
        end
    end

    figure;
    hold on;
    title('[LPAgrid] Estimate of start distances');
    h = bar3(flipud(lpa.v_mat));
    for i = 1:length(h)
         zdata = get(h(i),'Zdata');
         set(h(i),'Cdata',zdata)
    end
    axis([0 lpa.Nr+1 0 lpa.Nc+1]);
end

function key = CalcKey(s,lpa)

    g_val = getVecVal(s,lpa.g);
    rhs_val = getVecVal(s,lpa.rhs);

    [r,c] = ind2sub(size(lpa.A),s);

    key1 = min([g_val, rhs_val]) + lpa.heur([r,c]);
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
        fprintf('[LPAgrid] No such path can be created\n');
        lpa.path = {};
        lpa.path_plot = {};
        return;
    end

    lpa.path = {lpa.goal};

    vertex = lpa.goal;
    while vertex ~= lpa.start
        [Nbors,costs] = lpa.succ(vertex,lpa.A);

        lim = inf;
        key = 0;
        for k = 1:length(Nbors)
            g_val = getVecVal(Nbors{k},lpa.g);
            val = g_val + costs(k);
            if val < lim
                lim = val;
                key = k;
            end
        end

        vertex = Nbors{key};
        lpa.path{end+1} = vertex;
    end

    lpa.path = lpa.path(end:-1:1);

    for i = 1:length(lpa.path)
        [r,c] = ind2sub([lpa.Nr,lpa.Nc],lpa.path{i});
        lpa.path{i} = [r,c];
        lpa.path_plot{i} = [c,lpa.Nr-r+1];
    end
    
    fprintf('[LPAgrid] Created optimal path of length %d\n',g_goal);
end

function findpath(lpa)
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
        
%         disp(lpa.U);
        u = lpa.U.Pop();
        cnt = cnt + 1;
%         fprintf('Extracted vertex %d\n',u);
%         keyboard

        
        g_u = getVecVal(u,lpa.g);
        rhs_u = getVecVal(u,lpa.rhs);
        
        if g_u > rhs_u
            lpa.g = setVecVal(u,lpa.g,rhs_u);
            [Nbors,~] = lpa.succ(u,lpa.A);
            for k = 1:length(Nbors)
                lpa.UpdateVertex(Nbors{k});
            end
        else
            lpa.g = setVecVal(u,lpa.g,inf);
            [Nbors,~] = lpa.succ(u,lpa.A);
            Nbors{end+1} = u;
            for k = 1:length(Nbors)
                lpa.UpdateVertex(Nbors{k});
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
%         fprintf('\n');
    end
    
    fprintf('[LPAgrid] Finished in %d set extractions\n',cnt);
    
    if cnt ~= 0
        lpa.CreatePath();
    end
    
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
end
