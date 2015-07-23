function [ d ] = diagh( vert1,vert2,Nr,Nc )

[vr,vc] = ind2sub([Nr, Nc], vert1);
[gr,gc] = ind2sub([Nr, Nc], vert2);

dx = abs( vc - gc );
dy = abs (vr - gr );

D2 = 1;
D = 1;
d = D * (dx + dy) + (D2 - 2 * D) * min(dx, dy);

end

