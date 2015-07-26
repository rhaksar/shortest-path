function [ Nbors,costs ] = maze8_succ( x,A,h,p,q )
[n,m] = size(A);

r = x(1);
c = x(2);

Nbors = {};
costs = [];

if r + 1 <= n && A(r+1,c)
    Nbors{end+1} = [r+1,c];
    costs = [costs, p + h([r+1,c]) - h([r,c])];
end
if r - 1 >= 1 && A(r-1,c) 
    Nbors{end+1} = [r-1,c];
    costs = [costs, p + h([r-1,c]) - h([r,c])];
end

if c + 1 <= m && A(r,c+1)
    Nbors{end+1} = [r,c+1];
    costs = [costs, q + h([r,c+1]) - h([r,c])];
end
if c - 1 >= 1 && A(r,c-1)
    Nbors{end+1} = [r,c-1];
    costs = [costs, q + h([r,c-1]) - h([r,c])];
end

if r+1 <= n && c+1 <= m && A(r+1,c+1) 
    Nbors{end+1} = [r+1,c+1];
    costs = [costs, p + q + h([r+1,c+1]) - h([r,c])];
end
if r-1 >= 1 && c-1 >= 1 && A(r-1,c-1) 
    Nbors{end+1} = [r-1,c-1];
    costs = [costs, p + q + h([r-1,c-1]) - h([r,c])];
end

if r+1 <= n && c-1 >= 1 && A(r+1,c-1) 
    Nbors{end+1} = [r+1,c-1];
    costs = [costs, p + q + h([r+1,c-1]) - h([r,c])];
end
if r-1 >= 1 && c+1 <= m && A(r-1,c+1)
    Nbors{end+1} = [r-1,c+1];
    costs = [costs, p + q + h([r-1,c+1]) - h([r,c])];
end

end

