function [ d ] = diagh( s1,s2,p,q )

dx = p*abs( s1(2) - s2(2) );
dy = q*abs( s1(1) - s2(1) );

D2 = 1;
D = 1;
d = D * (dx + dy) + (D2 - 2 * D) * min(dx, dy);

end

