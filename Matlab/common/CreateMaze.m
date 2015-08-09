function [ start,goal,A,p,q,r ] = CreateMaze(maze_choice)


if maze_choice == 1
    load('maze_data.mat');
    start = sub2ind(size(A),s(1),s(2));
    goal = sub2ind(size(A),t(1),t(2));
    r = inf;
    A(55,36) = 2;
%     A(A == 0) = 1;
    return;
    
elseif maze_choice == 2
    A =  [-1	-1	0	0	-1	0	0	0	0	0	-1	-1	-1	0	0	-1	0	-1	-1	-1
        0	0	-1	0	-1	-1	-1	0	0	-1	0	0	-1	-1	0	-1	0	0	0	0
        -1	0	0	0	-1	0	-1	-1	0	0	0	0	0	-1	0	0	-1	-1	-1	0
        0	0	0	0	-1	0	-1	-1	0	0	0	0	-1	-1	0	-1	-1	0	0	-1
        0	0	0	-1	0	0	0	-1	-1	0	-1	0	-1	-1	0	-1	0	-1	0	0
        0	-1	-1	-1	-1	-1	0	-1	-1	-1	-1	0	-1	0	0	-1	0	-1	0	-1
        0	-1	-1	-1	-1	-1	0	0	0	0	0	0	-1	-1	-1	0	0	-1	0	0
        0	0	0	0	0	0	0	-1	-1	-1	-1	0	0	0	0	0	0	0	0	-1
        0	0	-1	-1	-1	-1	0	0	0	0	-1	-1	-1	0	-1	-1	-1	-1	0	-1
        -1	0	0	-1	-1	-1	-1	0	0	0	0	-1	-1	0	-1	0	0	-1	0	0
        -1	0	-1	-1	0	0	0	0	0	0	-1	-1	0	0	0	0	-1	-1	0	0
        0	0	0	-1	0	0	0	0	0	0	0	0	0	0	0	-1	0	-1	0	-1
        -1	0	0	0	0	-1	0	-1	0	0	-1	0	0	0	0	-1	0	0	0	0
        0	0	0	0	-1	-1	0	-1	0	-1	0	0	0	0	0	-1	0	0	-1	0
        -1	0	0	-1	0	-1	0	-1	-1	0	0	0	0	-1	-1	0	-1	0	-1	0];
    
    A(A == 0) = 1;
    A(A == -1) = 0;
%     A(A == -1) = 1;
    
%     A(8,13) = 2;
    %     A(8,6) = 2;
%     A(9,14) = 2;
%     A(7,1) = 2;
    %     A(9,19) = 2;
    p = 1;
    q = 1;
    r = inf;
    
    A = fliplr(A');
    
    start = sub2ind(size(A),4,8);
    goal = sub2ind(size(A),17,8);
    
%         start = 53;
%         goal = 248;
    return;
    
elseif maze_choice == 3
    A =  [-1	-1	0	0	-1	0	0	0	0	0	-1	-1	-1	0	0	-1	0	-1	-1	-1
        0	0	-1	0	-1	-1	-1	0	0	-1	0	0	-1	-1	0	-1	0	0	0	0
        -1	0	0	0	-1	0	-1	-1	0	0	0	0	0	-1	0	0	-1	-1	-1	0
        0	0	0	0	-1	0	-1	-1	0	0	0	0	-1	-1	0	-1	-1	0	0	-1
        0	0	0	-1	0	0	0	-1	-1	0	-1	0	-1	-1	0	-1	0	-1	0	0
        0	-1	-1	-1	-1	-1	0	-1	-1	-1	-1	0	-1	0	0	-1	0	-1	0	-1
        0	-1	-1	-1	-1	-1	0	0	0	0	0	0	-1	-1	-1	0	0	-1	0	0
        0	0	0	0	0	0	0	-1	-1	-1	-1	0	0	0	0	0	0	0	0	-1
        0	0	-1	-1	-1	-1	0	0	0	0	-1	-1	-1	0	-1	-1	-1	-1	0	-1
        -1	0	0	-1	-1	-1	-1	0	0	0	0	-1	-1	0	-1	0	0	-1	0	0
        -1	0	-1	-1	0	0	0	0	0	0	-1	-1	0	0	0	0	-1	-1	0	0
        0	0	0	-1	0	0	0	0	0	0	0	0	0	0	0	-1	0	-1	0	-1
        -1	0	0	0	0	-1	0	-1	0	0	-1	0	0	0	0	-1	0	0	0	0
        0	0	0	0	-1	-1	0	-1	0	-1	0	0	0	0	0	-1	0	0	-1	0
        -1	0	0	-1	0	-1	0	-1	-1	0	0	0	0	-1	-1	0	-1	0	-1	0];
    
    A(A == 0) = 1;
    A(A == -1) = 2;
    A(8,13) = 2;
%     A(8,6) = 2;
    A(9,14) = 2;
    
    A = fliplr(A');
    
    
    start = sub2ind(size(A),4,8);
    goal = sub2ind(size(A),17,8);
    p = 1;
    q = 1;
    r = inf;
    return;
    
elseif maze_choice == 4
    load('maze_data.mat');
    start = sub2ind(size(A),s(1),s(2));
    goal = sub2ind(size(A),t(1),t(2));
%     A(A == 0) = 2;
    A(A == 0) = 1;
    r = inf;
    
    return;
    
elseif maze_choice == 5
    A = [1, 1, 1, 1;
         0, 1, 0, 1;
         0, 1, 0, 1;
         0, 1, 0, 1;
         0, 1, 0, 1;
         1, 1, 1, 1];
     A = fliplr(A');
     start = 24;
     goal = 1;
     p = 1;
     q = 1;
     r = 1;
     return;
    
elseif maze_choice == 6
    A = [1 1 1;
         1 0 1;
         1 0 1;
         1 2 1;
         1 1 1];
     
     A = fliplr(A');
     start = 5;
     goal = 12;
     p = 1;
     q = 1;
     r = inf;
     return;
elseif maze_choice == 7
    A = ones(20,20);
    start = sub2ind(size(A),6,10);
    goal = sub2ind(size(A),16,10);
    p = 1;
    q = 1;
    r = inf;
    return;
elseif maze_choice == 8
    A = ones(5,5);
    start = 21;
    goal = 13;
    p = 1; 
    q = 1;
    r = inf;
    return;

    
else
    fprintf('Bad maze choice\n');
    A = [];
    start = -1;
    goal = -1;
    p = inf;
    q = inf;
    r = inf;
    return;
end


end

