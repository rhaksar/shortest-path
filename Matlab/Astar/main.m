%% A* with/without heuristics
clear; clc; close all;

[start,goal,A,p,q,r] = CreateMaze(1); 
[sr,sc] = ind2sub(size(A),start);
[gr,gc] = ind2sub(size(A),goal);
[Nr,Nc] = size(A);

heur = @(x) 0;  
% heur = @(x) diagh(x,[gr,gc]);
% heur = @(x) maxh(x,[gr,gc]);
% heur = @(x) manh(x,[gr,gc],p,q);

neighbors = @(x) maze4_neighbors( x,A,heur,p,q );
% neighbors = @(x) maze8_neighbors( x,A,heur,p,q,r );

[dist,path,cnt,v] = dijkstra( neighbors,start,goal,heur,Nr,Nc );
    
fprintf('Total set extractions: %d\n',cnt);
fprintf('Total path length: %d\n',dist);

for k = 1:length(path)
    [r,c] = ind2sub(size(A),path{k});
    path{k} = [r,c];
end

% showValues(v,A);
showPath(path,A);

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
neighbors = @(x) maze4_neighbors( x,A,heur,p,q );

[dist,path,cnt,v] = dijkstra( neighbors,start,goal,heur,Nr,Nc );

fprintf('Total set extractions: %d\n',cnt);
fprintf('Total path length: %d\n',dist);

for k = 1:length(path)
    [r,c] = ind2sub(size(A),path{k});
    path{k} = [r,c];
end
% showValues(v,A);
showPath(path,A);

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
            [dist,~,cnt,~] = dijkstra( neighbors, si, tj, heur, Nr, Nc );
            total = total + cnt;
            Gw(i,j) = dist;% + manh( W(i,:),W(j,:),p,q );
            
        end
    end
end
Dmanh = allPairsSP(Gw);
fprintf('Total set extractions to compute way-point distances: %d\n', total);

heur = @(x) heurMap(x,t,X,Y',W,Dmanh,p,q);
neighbors = @(x) maze4_neighbors( x,A,heur,p,q );

[dist,path,cnt,v] = dijkstra( neighbors,start,goal,heur,Nr,Nc );

fprintf('Total set extractions: %d\n',cnt);
fprintf('Total path distance: %d\n',dist);

for k = 1:length(path)
    [r,c] = ind2sub(size(A),path{k});
    path{k} = [r,c];
end

% showValues(v,A);
showPath(path,A);
