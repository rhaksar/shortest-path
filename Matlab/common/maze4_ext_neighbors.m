function [ Nbors,costs ] = maze4_ext_neighbors( x,A,h,p,q )

[n,m] = size(A);
[r,c] = ind2sub([n,m],x);

Nbors = {};
costs = [];
t = 1;

if r + 1 <= n && A(r+1,c) >= t
    Nbors{end+1} = [r+1,c];
    if A(r,c) >= t
        costs = [costs, p + h([r+1,c]) - h([r,c])];
    else
        costs = [costs, inf];
    end
end
if r - 1 >= 1 && A(r-1,c) >= t
    Nbors{end+1} = [r-1,c];
    if A(r,c) >= t
        costs = [costs, p + h([r-1,c]) - h([r,c])];
    else
        costs = [costs, inf];
    end
    
end

if c + 1 <= m && A(r,c+1) >= t
    Nbors{end+1} = [r,c+1];
    if A(r,c) >= t
        costs = [costs, q + h([r,c+1]) - h([r,c])];
    else
        costs = [costs, inf];
    end
end
if c - 1 >= 1 && A(r,c-1) >= t
    Nbors{end+1} = [r,c-1];
    if A(r,c) >= t
        costs = [costs, q + h([r,c-1]) - h([r,c])];
    else
        costs = [costs, inf];
    end
end


for i = 1:length(Nbors)
    vec = Nbors{i};
    ind = sub2ind([n,m],vec(1),vec(2));
    Nbors{i} = ind;
end



end

