clear; clc; close all;

[start,goal,A] = CreateMaze();
[Nr,Nc] = size(A);

heur = @(x,y) diagh(x,y,Nr,Nc);
succ = @(x,A) maze8_neighbors(x,A);
pred = @(x,A) maze8_neighbors(x,A);
obs = @(x,A) maze8_observer(x,A);
cost = @(x,y,A) maze8_cost(x,y,A);

dstar_obj = Dgrid(A,start,goal,succ,pred,obs,heur,cost);
dstar_obj.NavigateGrid(1);
