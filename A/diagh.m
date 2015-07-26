function [ d ] = diagh( s1,s2)

dx = abs( s1(2) - s2(2) );
dy = abs( s1(1) - s2(1) );

D2 = 1;
D = 1;
d = D * (dx + dy) + (D2 - 2 * D) * min(dx, dy);

end

