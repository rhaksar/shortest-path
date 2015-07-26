%% A* without heuristics
clear; clc; close all;

%%%
[start,goal,A,p,q] = CreateMaze(1);
heur = @(x) 0;
neighbors = @(x) maze4_dijkstra( x,A,heur,p,q );

% [start,goal,A,p,q] = CreateMaze(2); 
% heur = @(x) 0; 
% neighbors = @(x) maze8_dijkstra( x,A,heur,p,q ); 

total = 0;
[~,path,cnt] = dijkstra( neighbors,start,goal );
total = total + cnt;

path_final = {start};
r_start = path_final{1};
idx = 2;
while r_start ~= goal
    vs = maze4_observer(r_start,A); %%%
    if ~isempty(vs)
        for k = 1:length(vs)
            [r,c] = ind2sub(size(A),vs(k));
            A(r,c) = 0;
        end
        neighbors = @(x) maze4_dijkstra( x,A,heur,p,q ); %%%
        [~,path,cnt] = dijkstra( neighbors,r_start,goal );
        total = total + cnt;
        idx = 2;
        
        if size(path) < 2
            error('There is no possible path');
        end
    end
    
    r_start = path{idx};
    path_final{end+1} = r_start;
    
    idx = idx + 1;
end
fprintf('Created final path\n');
fprintf('Total set extractions: %d\n',total);

for k = 1:length(path_final)
    [r,c] = ind2sub(size(A),path_final{k});
    path_final{k} = [r,c];
end

% showPath(path_final,A);

%% A* with heuristic (manhattan or diagonal distance)
clear; clc; close all;

[start,goal,A,p,q] = CreateMaze(1);
[tr,tc] = ind2sub(size(A),goal);
heur = @(x) manh( x,[tr,tc],p,q );
neighbors = @(x) maze4_dijkstra( x,A,heur,p,q );

% [start,goal,A,p,q] = CreateMaze(2); 
% [Nr,Nc] = size(A);
% [tr,tc] = ind2sub(size(A),goal);
% heur = @(x) diagh( x,[tr,tc] );
% neighbors = @(x) maze8_dijkstra( x,A,heur,p,q );

total = 0;
[~,path,cnt] = dijkstra( neighbors,start,goal );
total = total + cnt;

path_final = {start};
r_start = path_final{1};
idx = 2;
while r_start ~= goal
    vs = maze4_observer(r_start,A);
    if ~isempty(vs)
        for k = 1:length(vs)
            [r,c] = ind2sub(size(A),vs(k));
            A(r,c) = 0;
        end
        neighbors = @(x) maze4_dijkstra( x,A,heur,p,q );
        [~,path,cnt] = dijkstra( neighbors,r_start,goal );
        total = total + cnt;
        idx = 2;
        
        if size(path) < 2
            error('There is no possible path');
        end
    end
    
    r_start = path{idx};
    path_final{end+1} = r_start;
    
    idx = idx + 1;
end
fprintf('Created final path\n');
fprintf('Total set extractions: %d\n',total);

for k = 1:length(path_final)
    [r,c] = ind2sub(size(A),path_final{k});
    path_final{k} = [r,c];
end

% showPath(path_final,A);

%% A* with heuristic (way-point manhattan distance)
clear; clc; close all;

load('maze_data.mat');
[start,goal,A,p,q] = CreateMaze(1);

[n,~] = size(W);
Gw = zeros(n,n);
for i = 1:n
    for j = 1:n
        if G(i,j) 
            Gw(i,j) = manh( W(i,:),W(j,:),p,q );
        end
    end
end
Dmanh = allPairsSP(Gw);

heur = @(x) heurMap(x,t,X,Y',W,Dmanh,p,q);
neighbors = @(x) maze4_dijkstra(x,A,heur,p,q);

total = 0;
[~,path,cnt] = dijkstra( neighbors,start,goal );
total = total + cnt;

path_final = {start};
r_start = path_final{1};
idx = 2;
while r_start ~= goal
    vs = maze4_observer(r_start,A);
    if ~isempty(vs)
        for k = 1:length(vs)
            [r,c] = ind2sub(size(A),vs(k));
            A(r,c) = 0;
        end
        neighbors = @(x) maze4_dijkstra( x,A,heur,p,q );
        [~,path,cnt] = dijkstra( neighbors,r_start,goal );
        total = total + cnt;
        idx = 2;
        
        if size(path) < 2
            error('There is no possible path');
        end
    end
    
    r_start = path{idx};
    path_final{end+1} = r_start;
    
    idx = idx + 1;
end
fprintf('Created final path\n');
fprintf('Total set extractions: %d\n',total);

for k = 1:length(path_final)
    [r,c] = ind2sub(size(A),path_final{k});
    path_final{k} = [r,c];
end

% showPath(path_final,A);

%% A* with heuristic (actual way-point distance)
clear; clc; close all;

load('maze_data.mat');
[start,goal,A,p,q] = CreateMaze(1);
[Nr,Nc] = size(A);
[tr,tc] = ind2sub(size(A),goal);

total = 0;
[n,~] = size(W);
Gw = zeros(n,n);
for i = 1:n
    for j = 1:n
        if G(i,j)
            si = sub2ind(size(A),W(i,1),W(i,2));
            tj = sub2ind(size(A),W(j,1),W(j,2));
            
            heur = @(x) manh( x,W(j,:),p,q );
            neighbors = @(x) maze4_dijkstra( x,A,heur,p,q );
            [dist,~,cnt] = dijkstra( neighbors, si, tj );
            total = total + cnt;
            Gw(i,j) = dist + manh( W(i,:),W(j,:),p,q );
            
        end
    end
end
Dmanh = allPairsSP(Gw);
fprintf('Total set extractions to compute way-point distances: %d\n',total);
total = 0;

heur = @(x) heurMap(x,t,X,Y',W,Dmanh,p,q);
neighbors = @(x) maze4_dijkstra(x,A,heur,p,q);

[~,path,cnt] = dijkstra( neighbors,start,goal );
total = total + cnt;

path_final = {start};
r_start = path_final{1};
idx = 2;
while r_start ~= goal
    vs = maze4_observer(r_start,A);
    if ~isempty(vs)
        for k = 1:length(vs)
            [r,c] = ind2sub(size(A),vs(k));
            A(r,c) = 0;
        end
        neighbors = @(x) maze4_dijkstra( x,A,heur,p,q );
        [~,path,cnt] = dijkstra( neighbors,r_start,goal );
        total = total + cnt;
        idx = 2;
        
        if size(path) < 2
            error('There is no possible path');
        end
    end
    
    r_start = path{idx};
    path_final{end+1} = r_start;
    
    idx = idx + 1;
end
fprintf('Created final path\n');
fprintf('Total set extractions: %d\n',total);

for k = 1:length(path_final)
    [r,c] = ind2sub(size(A),path_final{k});
    path_final{k} = [r,c];
end

showPath(path_final,A);
