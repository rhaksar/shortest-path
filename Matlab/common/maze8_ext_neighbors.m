function [ Nbors,costs ] = maze8_ext_neighbors( x,A,h,pc,qc,rc )

[Nr,Nc] = size(A);
[r,c] = ind2sub([Nr,Nc],x);

Nbors = {};
costs = [];
t = 1;

if r + 1 <= Nr && A(r+1,c) >= t
    Nbors{end+1} = [r+1,c];
    if A(r,c) >= t
        costs = [costs, pc + h([r+1,c]) - h([r,c])];
    else
        costs = [costs, inf];
    end
end
if r - 1 >= 1 && A(r-1,c) >= t
    Nbors{end+1} = [r-1,c];
    if A(r,c) >= t
        costs = [costs, pc + h([r-1,c]) - h([r,c])];
    else
        costs = [costs, inf];
    end
    
end

if c + 1 <= Nc && A(r,c+1) >= t
    Nbors{end+1} = [r,c+1];
    if A(r,c) >= t
        costs = [costs, qc + h([r,c+1]) - h([r,c])];
    else
        costs = [costs, inf];
    end
end
if c - 1 >= 1 && A(r,c-1) >= t
    Nbors{end+1} = [r,c-1];
    if A(r,c) >= t
        costs = [costs, qc + h([r,c-1]) - h([r,c])];
    else
        costs = [costs, inf];
    end
end


if r+1 <= Nr && c+1 <= Nc && A(r+1,c+1) >= t
    Nbors{end+1} = [r+1,c+1];
    if A(r,c) >= t
        costs = [costs, rc + h([r+1,c+1]) - h([r,c])];
    else
        costs = [costs, inf];
    end
end

if r-1 >= 1 && c-1 >= 1 && A(r-1,c-1) >= t
    Nbors{end+1} = [r-1,c-1];
    if A(r,c) >= t
        costs = [costs, rc + h([r-1,c-1]) - h([r,c])];
    else
        costs = [costs, inf];
    end
end

if r+1 <= Nr && c-1 >= 1 && A(r+1,c-1) >= t
    Nbors{end+1} = [r+1,c-1];
    if A(r,c) >= t
        costs = [costs, rc + h([r+1,c-1]) - h([r,c])];
    else
        costs = [costs, inf];
    end
end

if r-1 >= 1 && c+1 <= Nc && A(r-1,c+1) >= t
    Nbors{end+1} = [r-1,c+1];
    if A(r,c) >= t
        costs = [costs, rc + h([r-1,c+1]) - h([r,c])];
    else
        costs = [costs, inf];
    end
end

for i = 1:length(Nbors)
    vec = Nbors{i};
    ind = sub2ind([Nr,Nc],vec(1),vec(2));
    Nbors{i} = ind;
end

end

