function [ h ] = manh( x,t,Nr,Nc,p,q )

[xr,xc] = ind2sub([Nr, Nc], x);
[tr,tc] = ind2sub([Nr, Nc], t);

h = p*abs( xr - tr ) + q*abs( xc - tc );

end

