function [ Nbors,costs ] = stock_ext_pred( x,A,ps )

[Nr,Nc] = size(A);
[r,c] = ind2sub([Nr,Nc],x);

Nbors = {};
costs = [];
t = 1;

if c-1 >= 1 && A(r,c-1) >= t && c ~= Nc
    Nbors{end+1} = [r,c-1];
    
    if A(r,c) >= t
        costs = [costs, 0];
    else
        costs = [costs, inf];
    end
end

if r+1 <= Nr && c-1 >= 1 && A(r+1,c-1) >= t && c ~= Nc
    Nbors{end+1} = [r+1,c-1];
    
    if A(r,c) >= t
        costs = [costs, 1*ps(c-1)];
    else
        costs = [costs, inf];
    end
end

if r == 1 && c == Nc
    for i = 1:Nr
        Nbors{end+1} = [i,c-1];
        costs = [costs,0];
    end
end

for i = 1:length(Nbors)
    vec = Nbors{i};
    ind = sub2ind([Nr,Nc],vec(1),vec(2));
    Nbors{i} = ind;
end


end

