%% Exiting the market with incomplete fulfillment
clear; clc; close all;

T = 50;
S = 10;
p = 0.6;

w1 = 0.6:0.1:2;
mu = 0;
sigma = 0.2;
w1dist = lognpdf(w1,mu,sigma)/sum(lognpdf(w1,mu,sigma));

% prices = zeros(1,T);
% for i = 1:T
%     x = sample(w1dist);
%     prices(i) = w1(x);
% end

Ex = sum(w1.*w1dist);
prices = Ex*ones(1,T);

A = ones(S+1,T+1);
A(2:end,end) = 0;
% A = fliplr(A');

start = S+1;
goal = sub2ind(size(A),1,T+1);
% start = 1;
% goal = sub2ind(size(A),T+1,S+1);

heur1 = @(x,y) 0;
% heur2 = @(x,y) manh(x,y,p,q);
% heur3 = @(x,y) diagh(x,y);
% heur4 = @(x,y) maxh(x,y,p,q);

succ = @(x,A) stock_ext_succ(x,A,prices);
pred = @(x,A) stock_ext_pred(x,A,prices);
% obs = @(x,A) [];
obs = @(x,A) stock_obs(x,A,p);

dstar_obj = Dgrid_max(A,start,goal,succ,pred,obs,heur1);
cnt = dstar_obj.ComputeShortestPath();

prices = zeros(1,T);
for i = 1:T
    x = sample(w1dist);
    prices(i) = w1(x);
end

succ = @(x,A) stock_ext_succ(x,A,prices);
pred = @(x,A) stock_ext_pred(x,A,prices);
dstar_obj.succ = succ;
dstar_obj.pred = pred;

[total_cnt,total_dist] = dstar_obj.NavigateGrid(0);
fprintf('Number of extractions: %d\n',total_cnt + cnt);
fprintf('Length of path: %0.2f\n',total_dist);
showPath2(dstar_obj.path,dstar_obj.A);

% NMC = 1;
% total_cnt = zeros(1,NMC);
% total_dist = zeros(1,NMC);
% for n = 1:NMC
%     fprintf('Iteration: %d\n',n);
% 
%     prices = Ex*ones(1,T);
%     succ = @(x,A) stock_ext_succ(x,A,prices);
%     pred = @(x,A) stock_ext_pred(x,A,prices);
%     
%     dstar_obj = Dgrid_max(A,start,goal,succ,pred,obs,heur1);
%     cnt = dstar_obj.ComputeShortestPath();
% 
%     prices = zeros(1,T);
%     for i = 1:T
%         x = sample(w1dist);
%         prices(i) = w1(x);
%     end
% 
%     succ = @(x,A) stock_ext_succ(x,A,prices);
%     pred = @(x,A) stock_ext_pred(x,A,prices);
%     dstar_obj.succ = succ;
%     dstar_obj.pred = pred;
%     [total_cnt(n),total_dist(n)] = dstar_obj.NavigateGrid(1);
%     total_cnt(n) = total_cnt(n) + cnt;
% %     clear dstar_obj
% %     fprintf('\n');
% end
% 
% avg_cnt = mean(total_cnt);
% avg_dist = mean(total_dist);
% 
% fprintf('The average number of extractions is: %0.2f\n',avg_cnt);
% fprintf('The average path length is: %0.2f\n', avg_dist);