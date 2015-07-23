clear; clc; close all;

% [start,goal,A] = CreateMaze();
% [Nr,Nc] = size(A);

load('maze_data.mat');
[Nr,Nc] = size(A);
start = sub2ind(size(A),s(1),s(2));
goal = sub2ind(size(A),t(1),t(2));

heur = @(x,y) manh(x,y,Nr,Nc,p,q);
% heur = @(x,y) diagh(x,y,Nr,Nc);
succ = @(x,A) maze4_neighbors(x,A);
pred = @(x,A) maze4_neighbors(x,A);
cost = @(x,y,A) maze4_cost(x,y,A,p,q);

lpa_obj = LPAgrid(A,start,goal,succ,pred,heur,cost);

run = 1;
while run
    lpa_obj.ComputeShortestPath();
    lpa_obj.CreatePath();
    showPath(lpa_obj.path,A);
    %lpa_obj.ShowPath();
    
    while true
        x = input('''1'' to change the maze, ''2'' to exit: ');
        if x == 1
            %[vertices,lpa_obj.A] = ChangeMaze();
            lpa_obj.A(55,36) = 0;
            idx = sub2ind([Nr,Nc],55,36);
            vertices = [idx succ(idx,lpa_obj.A)];
            for k = vertices
                lpa_obj.UpdateVertex(k);
            end
            break;
        elseif x == 2
            fprintf('Done!\n');
            run = 0;
            break;
        end
    end

end