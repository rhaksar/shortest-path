%% A* without heuristics
clear; clc; close all;

%%%
% [start,goal,A,p,q] = CreateMaze(1);
% heur = @(x) 0;
% neighbors = @(x) maze4_dijkstra( x,A,heur,p,q );

[start,goal,A,p,q] = CreateMaze(2); 
heur = @(x) 0; 
neighbors = @(x) maze8_dijkstra( x,A,heur,p,q ); 

[~,path] = dijkstra( neighbors,start,goal );

path_final = {start};
r_start = path_final{1};
idx = 2;
while r_start ~= goal
    vs = maze8_observer(r_start,A); %%%
    if ~isempty(vs)
        for k = 1:length(vs)
            [r,c] = ind2sub(size(A),vs(k));
            A(r,c) = 0;
        end
        neighbors = @(x) maze8_dijkstra( x,A,heur,p,q ); %%%
        [~,path] = dijkstra( neighbors,r_start,goal );
        idx = 2;
    end
    
    r_start = path{idx};
    path_final{end+1} = r_start;
    
    idx = idx + 1;
end
fprintf('Created final path\n');

for k = 1:length(path_final)
    [r,c] = ind2sub(size(A),path_final{k});
    path_final{k} = [r,c];
end

showPath(path_final,A);

%% A* with heuristic (manhattan or diagonal distance)
clear; clc; close all;

% [start,goal,A,p,q] = CreateMaze(1);
% [Nr,Nc] = size(A);
% [tr,tc] = ind2sub(size(A),goal);
% heur = @(x) manh( x,[tr,tc],p,q );
% neighbors = @(x) maze4_dijkstra( x,A,heur,p,q );

[start,goal,A,p,q] = CreateMaze(2); 
[Nr,Nc] = size(A);
[tr,tc] = ind2sub(size(A),goal);
heur = @(x) diagh( x,[tr,tc] );
neighbors = @(x) maze8_dijkstra( x,A,heur,p,q );

[dist,path] = dijkstra( neighbors,start,goal );

path_final = {start};
r_start = path_final{1};
idx = 2;
while r_start ~= goal
    vs = maze8_observer(r_start,A);
    if ~isempty(vs)
        for k = 1:length(vs)
            [r,c] = ind2sub(size(A),vs(k));
            A(r,c) = 0;
        end
        neighbors = @(x) maze8_dijkstra( x,A,heur,p,q );
        [~,path] = dijkstra( neighbors,r_start,goal );
        idx = 2;
    end
    
    r_start = path{idx};
    path_final{end+1} = r_start;
    
    idx = idx + 1;
end
fprintf('Created final path\n');

for k = 1:length(path_final)
    [r,c] = ind2sub(size(A),path_final{k});
    path_final{k} = [r,c];
end

showPath(path_final,A);

%% A* with heuristic (way-point manhattan distance)

%% A* with heuristic (actual way-point distance)