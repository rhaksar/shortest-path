function [ d ] = maxh( s1,s2,p,q )
dx = abs( s1(2) - s2(2) );
dy = abs( s1(1) - s2(1) );

d = max([q*dx,p*dy]);

end

