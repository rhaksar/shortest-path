clear; clc; close all;

load('maze_data.mat');
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
heur = @(x,t) heurMap(x,t,X,Y',W,Dmanh,p,q);

[start,goal,A,p,q] = CreateMaze(1);

% heur = @(x,y) manh(x,y,p,q);
succ = @(x,A) maze4_neighbors(x,A);
pred = @(x,A) maze4_neighbors(x,A);
obs = @(x,A) maze4_observer(x,A);
cost = @(x,y,A) maze4_cost(x,y,A,p,q);

% heur = @(x,y) diagh(x,y,p,q);
% succ = @(x,A) maze8_neighbors(x,A);
% pred = @(x,A) maze8_neighbors(x,A);
% obs = @(x,A) maze8_observer(x,A);
% cost = @(x,y,A) maze8_cost(x,y,A,p,q);

dstar_obj = Dgrid(A,start,goal,succ,pred,obs,heur,cost);
dstar_obj.NavigateGrid(1);
