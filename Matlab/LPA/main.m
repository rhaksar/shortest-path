%% LPA* with/without heuristics
clear; clc; close all;

[start,goal,A,p,q,r] = CreateMaze(1);
[sr,sc] = ind2sub(size(A),start);
[gr,gc] = ind2sub(size(A),goal);

heur1 = @(x) 0;
heur2 = @(x) manh(x,[gr,gc],p,q);
% heur3 = @(x,y) diagh(x,y,p,q);

succ = @(x,A) maze4_ext_neighbors(x,A,heur1,p,q);
pred = @(x,A) maze4_ext_neighbors(x,A,heur1,p,q);

lpa_obj = LPAgrid(A,start,goal,succ,pred,heur1);
lpa_obj.ComputeShortestPath();
% lpa_obj.ShowPath();
% lpa_obj.ShowValues();

fprintf('Any key to continue...\n');
pause

lpa_obj.UpdateGrid();
lpa_obj.ComputeShortestPath();

% lpa_obj.ShowPath();
% lpa_obj.ShowValues();

%% LPA* with way-point Manhattan distance
clear; clc; close all;

load('maze_data.mat');
[start,goal,A,p,q,r] = CreateMaze(1);
[gr,gc] = ind2sub(size(A),goal);

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

heur1 = @(x) 0;
heur2 = @(x) heurMap(x,[gr,gc],X,Y',W,Dmanh,p,q);

succ = @(x,A) maze4_ext_neighbors(x,A,heur1,p,q);
pred = @(x,A) maze4_ext_neighbors(x,A,heur1,p,q);

lpa_obj = LPAgrid(A,start,goal,succ,pred,heur2);
lpa_obj.ComputeShortestPath();
% lpa_obj.ShowPath();

fprintf('Any key to continue...\n');
pause

lpa_obj.UpdateGrid();
lpa_obj.ComputeShortestPath();
lpa_obj.ShowPath();

%% LPA* with way-point actual distance
clear; clc; close all;

load('maze_data.mat');
[start,goal,A,p,q] = CreateMaze(1);
[Nr,Nc] = size(A);
[gr,gc] = ind2sub(size(A),goal);

total_cnt = 0;
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
            Gw(i,j) = dist;% + manh( W(i,:),W(j,:),p,q );
            total_cnt = total_cnt + cnt;
            
        end
    end
end
Dmanh = allPairsSP(Gw);
fprintf('Total set extractions to compute way-point distances: %d\n', total_cnt);

heur1 = @(x) 0;
heur2 = @(x) heurMap(x,[gr,gc],X,Y',W,Dmanh,p,q);
succ = @(x,A) maze4_ext_neighbors(x,A,heur1,p,q);
pred = @(x,A) maze4_ext_neighbors(x,A,heur1,p,q);

lpa_obj = LPAgrid(A,start,goal,succ,pred,heur2);
lpa_obj.ComputeShortestPath();
lpa_obj.ShowPath();

fprintf('Any key to continue ...\n');
pause;

lpa_obj.UpdateGrid();
lpa_obj.ComputeShortestPath();
lpa_obj.ShowPath();