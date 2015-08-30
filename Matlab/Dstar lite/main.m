%% D* Lite with/without heurisitics
clear; clc; close all;
[start,goal,A,p,q,r] = CreateMaze(2);
[gr,gc] = ind2sub(size(A),goal);

heur1 = @(x) 0;
heur2 = @(x,y) manh(x,y,p,q);
heur3 = @(x,y) diagh(x,y);
heur4 = @(x,y) maxh(x,y,p,q);
heur5 = @(x,y) 0;

% succ = @(x,A) maze4_ext_neighbors(x,A,heur1,p,q);
% pred = @(x,A) maze4_ext_neighbors(x,A,heur1,p,q);
% obs = @(x,A) maze4_observer(x,A);

pred = @(x,A) maze8_ext_neighbors(x,A,heur1,p,q,r);
succ = @(x,A) maze8_ext_neighbors(x,A,heur1,p,q,r);
obs = @(x,A) maze8_observer(x,A);

dstar_obj = Dgrid(A,start,goal,succ,pred,obs,heur3);
dstar_obj.NavigateGrid(1);


%% D* Lite with Manhattan way-point distances
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

heur1 = @(x) 0;
heur2 = @(x,t) heurMap(x,t,X,Y',W,Dmanh,p,q);

succ = @(x,A) maze4_ext_neighbors(x,A,heur1,p,q);
pred = @(x,A) maze4_ext_neighbors(x,A,heur1,p,q);
obs = @(x,A) maze4_observer(x,A);

dstar_obj = Dgrid(A,start,goal,succ,pred,obs,heur2);
dstar_obj.NavigateGrid(0);

%% D* Lite with actual way-point distances
clear; clc; close all;

load('maze_data.mat');
[Nr,Nc] = size(A);
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
            [dist,~,cnt,~] = dijkstra( neighbors,si,tj,heur,Nr,Nc );
            total = total + cnt;
            Gw(i,j) = dist;% + manh( W(i,:),W(j,:),p,q );
            
        end
    end
end
Dmanh = allPairsSP(Gw);
fprintf('Total set extractions to compute way-point distances: %d\n',total);
total = 0;

heur1 = @(x) 0;
heur2 = @(x,t) heurMap(x,t,X,Y',W,Dmanh,p,q);

[start,goal,A,p,q,r] = CreateMaze(1);

succ = @(x,A) maze4_ext_neighbors(x,A,heur1,p,q);
pred = @(x,A) maze4_ext_neighbors(x,A,heur1,p,q);
obs = @(x,A) maze4_observer(x,A);

dstar_obj = Dgrid(A,start,goal,succ,pred,obs,heur2);
dstar_obj.NavigateGrid(1);

%% Experimental stuff for Dstar Lite
% D* Lite for an unknown 8maze
clear; clc; close all;

[start,goal,A,p,q,r] = CreateMaze(2);

heur1 = @(x) 0;
heur2 = @(x,y) 0;
heur3 = @(x,y) diagh(x,y);

succ = @(x,A) maze8_ext_neighbors(x,A,heur1,p,q,r);
pred = @(x,A) maze8_ext_neighbors(x,A,heur1,p,q,r);
obs = @(x,A) maze8_observer(x,A);

dstar_obj = Dgrid(A,start,goal,succ,pred,obs,heur3);
dstar_obj.NavigateGrid(0);

dstar_obj.ShowGrid();

%% D* Lite with/without heuristics for an unknown 4maze
clear; clc; close all;

[start,goal,A,p,q,r] = CreateMaze(1);

heur1 = @(x) 0;
heur2 = @(x,y) 0;
heur3 = @(x,y) manh(x,y,p,q);
heur4 = @(x,y) diagh(x,y);
heur5 = @(x,y) maxh(x,y,p,q);

succ = @(x,A) maze4_ext_neighbors(x,A,heur1,p,q);
pred = @(x,A) maze4_ext_neighbors(x,A,heur1,p,q);
obs = @(x,A) maze4_observer(x,A);

dstar_obj = Dgrid(A,start,goal,succ,pred,obs,heur3);
dstar_obj.NavigateGrid(0);

% dstar_obj.ShowValues();
dstar_obj.ShowGrid();

%% D* Lite with Manhattan way-point distances for an unknown 4maze
clear; clc; close all;

load('maze_data.mat');
[start,goal,A,p,q,r] = CreateMaze(1);

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
heur2 = @(x,t) heurMap(x,t,X,Y',W,Dmanh,p,q);

succ = @(x,A) maze4_ext_neighbors(x,A,heur1,p,q);
pred = @(x,A) maze4_ext_neighbors(x,A,heur1,p,q);
obs = @(x,A) maze4_observer(x,A);

dstar_obj = Dgrid(A,start,goal,succ,pred,obs,heur2);
dstar_obj.NavigateGrid(0);

dstar_obj.ShowGrid();

%% D* Lite with actual way-point distances for an unknown 4maze
clear; clc; close all;

load('maze_data.mat');
[Nr,Nc] = size(A);
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
            [dist,~,cnt,~] = dijkstra( neighbors,si,tj,heur,Nr,Nc );
            total = total + cnt;
            Gw(i,j) = dist;% + manh( W(i,:),W(j,:),p,q );
            
        end
    end
end
Dmanh = allPairsSP(Gw);
fprintf('Total set extractions to compute way-point distances: %d\n',total);
total = 0;

heur1 = @(x) 0;
heur2 = @(x,t) heurMap(x,t,X,Y',W,Dmanh,p,q);

[start,goal,A,p,q,r] = CreateMaze(1);

succ = @(x,A) maze4_ext_neighbors(x,A,heur1,p,q);
pred = @(x,A) maze4_ext_neighbors(x,A,heur1,p,q);
obs = @(x,A) maze4_observer(x,A);

dstar_obj = Dgrid(A,start,goal,succ,pred,obs,heur2);
dstar_obj.NavigateGrid(0);

dstar_obj.ShowGrid();
