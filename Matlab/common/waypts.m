function [ wpind ] = waypts( x,X,Y,W )
% Inputs:
%   x : current maze position
%   X : location of vertical lines
%   Y : location of horizontal lines
%   W : indices of waypoints
% Outputs:
%   wpind : vector indicating reachable waypoints from x

n = size(W,1);
r = x(1);
c = x(2);

wpind = [];

for i = 1:n
    if r == W(i,1) && c == W(i,2)
        wpind = i;
        return;
    end
end

Xnew = sort([X r]);
Ynew = sort([Y c]);

ind_x = find(Xnew == r);
ind_y = find(Ynew == c);

% disp(ind_x);
% disp(ind_y);

if length(ind_x) > 1
    ind_x = ind_x(1);
end
if length(ind_y) > 1
    ind_y = ind_y(1);
end

if ind_x > length(X)
    x_ulim = inf;
    x_llim = Xnew(ind_x-1);
elseif ind_x == 1
    x_ulim = Xnew(ind_x+1);
    x_llim = 1;
else
    x_ulim = Xnew(ind_x+1);
    x_llim = Xnew(ind_x-1);
end

if ind_y > length(Y)
    y_ulim = inf;
    y_llim = Ynew(ind_y-1);
elseif ind_y == 1
    y_ulim = Ynew(ind_y+1);
    y_llim = 1;
else
    y_ulim = Ynew(ind_y+1);
    y_llim = Ynew(ind_y-1);
end

for k = 1:n
    i = W(k,1);
    j = W(k,2);
    
    if i >= x_llim && i <= x_ulim && j >= y_llim && j <= y_ulim
        wpind(end+1) = k;
    end
end


end

