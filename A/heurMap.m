function [ hx ] = heurMap( x,t,X,Y,W,D,p,q )
% Inputs:
%   x : current maze position
%   t : maze target position
%   X : location of vertical lines
%   Y : location of horizontal lines
%   W : indices of waypoints
%   D : estimates of distance between waypoints
%   p : cost to move horizontally
%   q : cost to move vertically
% Outputs:
%   hx : heuristic distance estimate

wp_x = waypts(x,X,Y,W);
wp_t = waypts(t,X,Y,W);

if sum(size(wp_x) - size(wp_t)) == 0 && sum(sum(wp_x - wp_t)) == 0
    hx = manh(x,t,p,q);
else
    lim = inf;
    for i = wp_x
        for j = wp_t
            
            res = manh(x,W(i,:),p,q) + D(i,j) + manh(W(j,:),t,p,q);
            if res < lim
                hx = res;
                lim = res;
            end
            
        end
    end
    
end

end

