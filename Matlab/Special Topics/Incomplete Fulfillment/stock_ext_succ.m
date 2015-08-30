function [ Nbors,costs ] = stock_ext_succ( x,A,ps )

[Nr,Nc] = size(A);
[r,c] = ind2sub([Nr,Nc],x);

Nbors = {};
costs = [];
t = 1;

if c+1 <= Nc && A(r,c+1) >= t && c ~= Nc-1
    Nbors{end+1} = [r,c+1];
    
    if A(r,c) >= t
        costs = [costs, 0];
    else
        costs = [costs, inf];
    end
end

if r-1 >= 1 && c+1 <= Nc && A(r-1,c+1) >= t && c ~= Nc-1
    Nbors{end+1} = [r-1,c+1];
    
    if A(r,c) >= t
        costs = [costs, 1*ps(c)];
    else
        costs = [costs, inf];
    end
end

if c == Nc-1
    Nbors{end+1} = [1,Nc];
    costs = [costs, 0];
end

for i = 1:length(Nbors)
    vec = Nbors{i};
    ind = sub2ind([Nr,Nc],vec(1),vec(2));
    Nbors{i} = ind;
end


end

