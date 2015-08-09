function [Nbors,costs] = maze8_dijkstra( x,A,h,pc,qc,rc )

[n,m] = size(A);
[r,c] = ind2sub([n,m],x);

Nbors = {};
costs = [];

if r + 1 <= n && A(r+1,c)
    Nbors{end+1} = [r+1,c];
    costs = [costs, pc + h([r+1,c]) - h([r,c])];
end
if r - 1 >= 1 && A(r-1,c) 
    Nbors{end+1} = [r-1,c];
    costs = [costs, pc + h([r-1,c]) - h([r,c])];
end

if c + 1 <= m && A(r,c+1)
    Nbors{end+1} = [r,c+1];
    costs = [costs, qc + h([r,c+1]) - h([r,c])];
end
if c - 1 >= 1 && A(r,c-1)
    Nbors{end+1} = [r,c-1];
    costs = [costs, qc + h([r,c-1]) - h([r,c])];
end

if r+1 <= n && c+1 <= m && A(r+1,c+1) 
    Nbors{end+1} = [r+1,c+1];
    costs = [costs, rc + h([r+1,c+1]) - h([r,c])];
end
if r-1 >= 1 && c-1 >= 1 && A(r-1,c-1) 
    Nbors{end+1} = [r-1,c-1];
    costs = [costs, rc + h([r-1,c-1]) - h([r,c])];
end

if r+1 <= n && c-1 >= 1 && A(r+1,c-1) 
    Nbors{end+1} = [r+1,c-1];
    costs = [costs, rc + h([r+1,c-1]) - h([r,c])];
end
if r-1 >= 1 && c+1 <= m && A(r-1,c+1)
    Nbors{end+1} = [r-1,c+1];
    costs = [costs, rc + h([r-1,c+1]) - h([r,c])];
end

for i = 1:length(Nbors)
    vec = Nbors{i};
    ind = sub2ind([n,m],vec(1),vec(2));
    Nbors{i} = ind;
end

end

