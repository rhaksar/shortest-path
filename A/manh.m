function [ h ] = manh( x,t,p,q )
% Inputs:
%   x : vector describing a maze position
%   t : target position in maze
%   p : cost to move horizontally
%   q : cost to move vertically


h = p*abs( x(1) - t(1) ) + q*abs( x(2) - t(2) );

end

