%% A* with/without heuristics
clear; clc; close all;

[start,goal,A,p,q,r] = CreateMaze(1);
[Nr,Nc] = size(A);
[sr,sc] = ind2sub(size(A),start);
[tr,tc] = ind2sub(size(A),goal);

heur1 = @(x) 0;
heur2 = @(x) manh( x,[tr,tc],p,q );
neighbors = @(x) maze4_neighbors( x,A,heur2,p,q );

% [start,goal,A,p,q] = CreateMaze(2); 
% heur = @(x) 0;  
% neighbors = @(x) maze8_dijkstra( x,A,heur,p,q,r ); 

total = 0;
total_dist = 0;
[dist,path,cnt,~] = dijkstra( neighbors,start,goal,heur2,Nr,Nc );
total = total + cnt;
% total_dist = total_dist + dist;

path_final = {start};
r_start = path_final{1};
idx = 2;
while r_start ~= goal
    [r,c] = ind2sub(size(A),path{idx});
    
    if A(r,c) == 2;
        A(r,c) = 0;
        neighbors = @(x) maze4_neighbors( x,A,heur2,p,q ); %%%
        
        [dist,path,cnt,~] = dijkstra( neighbors,r_start,goal,heur2,Nr,Nc );
%         total_dist = total_dist + dist;
        total = total + cnt;
        idx = 2;
        
        if size(path) < 2
            error('There is no possible path');
        end
    end
    
    
    [Nbors,costs] = maze4_neighbors( path{idx-1},A,heur1,p,q );
    r_start = path{idx};
    
    for i = 1:length(Nbors)
        if Nbors{i} == r_start
            total_dist = total_dist + costs(i);vv
            continue
        end
    end
    
    path_final{end+1} = r_start;
    
    idx = idx + 1;
end

fprintf('Total set extractions: %d\n',total);
fprintf('Total path distance: %d\n',total_dist);

for k = 1:length(path_final)
    [r,c] = ind2sub(size(A),path_final{k});
    path_final{k} = [r,c];
end

% showPath(path_final,A);

%% A* with heuristic (way-point manhattan distance)
clear; clc; close all;

load('maze_data.mat');
[start,goal,A,p,q] = CreateMaze(1);
[Nr,Nc] = size(A);
[sr,sc] = ind2sub(size(A),start);

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
neighbors = @(x) maze4_neighbors(x,A,heur,p,q);

total = 0;
total_dist = 0;

[dist,path,cnt] = dijkstra( neighbors,start,goal,heur,Nr,Nc );
total_dist = total_dist + dist;
total = total + cnt;

path_final = {start};
r_start = path_final{1};
idx = 2;
while r_start ~= goal
    [r,c] = ind2sub(size(A),path{idx});
    if A(r,c) == 2  
        A(r,c) = 0;
        neighbors = @(x) maze4_neighbors( x,A,heur,p,q );
        
        [dist,path,cnt] = dijkstra( neighbors,r_start,goal,heur,Nr,Nc );
        [r_sr,r_sc] = ind2sub(size(A),r_start);
        total_dist = total_dist + dist;
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
fprintf('Total set extractions: %d\n',total);
fprintf('Total path length: %d\n',total_dist);

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
[sr,sc] = ind2sub(size(A),start);
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
            neighbors = @(x) maze4_neighbors( x,A,heur,p,q );
            [dist,~,cnt] = dijkstra( neighbors,si,tj,heur,Nr,Nc );
            total = total + cnt;
            Gw(i,j) = dist;
            
        end
    end
end
Dmanh = allPairsSP(Gw);
fprintf('Total set extractions to compute way-point distances: %d\n',total);

heur = @(x) heurMap(x,t,X,Y',W,Dmanh,p,q);
neighbors = @(x) maze4_neighbors(x,A,heur,p,q);

total = 0;
total_dist = 0;

[dist,path,cnt] = dijkstra( neighbors,start,goal,heur,Nr,Nc );
total_dist = total_dist + dist;
total = total + cnt;

path_final = {start};
r_start = path_final{1};
idx = 2;
while r_start ~= goal
    [r,c] = ind2sub(size(A),path{idx});
    if A(r,c) == 2
        A(r,c) = 0;
        neighbors = @(x) maze4_neighbors( x,A,heur,p,q );
        
        [dist,path,cnt] = dijkstra( neighbors,r_start,goal,heur,Nr,Nc );
        [r_sr,r_sc] = ind2sub(size(A),r_start);
        total_dist = total_dist + dist;% + heur([r_sr,r_sc]);
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
fprintf('Total set extractions: %d\n',total);
fprintf('Total path length: %d\n',total_dist);

for k = 1:length(path_final)
    [r,c] = ind2sub(size(A),path_final{k});
    path_final{k} = [r,c];
end

% showPath(path_final,A);
