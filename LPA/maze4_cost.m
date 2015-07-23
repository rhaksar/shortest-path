function [ c ] = maze4_cost( v1,v2,A,p,q )
[Nr,Nc] = size(A);
[r1,c1] = ind2sub([Nr,Nc],v1);
[r2,c2] = ind2sub([Nr,Nc],v2);

dr = abs(r1 - r2);
dc = abs(c1 - c2);

thresh = 1;

if dr == 1 && dc == 0 && A(r1,c1) >= thresh && A(r2,c2) >= thresh
    c = p;
elseif dr == 0 && dc == 1 && A(r1,c1) >= thresh && A(r2,c2) >= thresh
    c = q;
else
    c = inf;
end

end

