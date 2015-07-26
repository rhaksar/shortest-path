function [Nbors,costs] = maze8_dijkstra( x,A,h,p,q )

[n,m] = size(A);
[r,c] = ind2sub([n,n],x);

[Nbors, costs] = maze8_succ([r,c],A,h,p,q);

for i = 1:length(Nbors)
    vec = Nbors{i};
    ind = sub2ind([n,m],vec(1),vec(2));
    Nbors{i} = ind;
end

end

