%% LPA* 
clear; clc; close all;

[start,goal,A,p,q] = CreateMaze(1);
[Nr,Nc] = size(A);

heur = @(x,y) manh(x,y,p,q);
% heur = @(x,y) diagh(x,y,p,q);
succ = @(x,A) maze4_neighbors(x,A);
pred = @(x,A) maze4_neighbors(x,A);
cost = @(x,y,A) maze4_cost(x,y,A,p,q);

lpa_obj = LPAgrid(A,start,goal,succ,pred,heur,cost);
lpa_obj.ComputeShortestPath();
lpa_obj.CreatePath();
lpa_obj.ShowPath();
keyboard

vs = find(lpa_obj.A == 2);
upd_vs = [];
for k = 1:length(vs)
    [r,c] = ind2sub(size(lpa_obj.A),vs(k));
    lpa_obj.A(r,c) = 0; 
    upd_vs = [upd_vs vs(k) succ(vs(k),lpa_obj.A)];
end
for k = upd_vs
    lpa_obj.UpdateVertex(k);
end

lpa_obj.ComputeShortestPath();
lpa_obj.CreatePath();
lpa_obj.ShowPath();