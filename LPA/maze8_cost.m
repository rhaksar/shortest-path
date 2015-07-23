function [ c ] = maze8_cost( v1,v2,A )

[Nr,Nc] = size(A);
[r1,c1] = ind2sub([Nr,Nc],v1);
[r2,c2] = ind2sub([Nr,Nc],v2);

dx = abs(r1 - r2);
dy = abs(c1 - c2);

thresh = 1;

if dx <= 1 && dy <= 1 && A(r1,c1) >= thresh && A(r2,c2) >= thresh
    c = 1;
else
    c = inf;
end


end

