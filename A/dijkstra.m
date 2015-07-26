function [ dist,path ] = dijkstra( neighbors,s,t )
% Inputs:
%   neighbors : function handle to determine neighbors of a vertex
%   s : source vertex
%   v : destination vertex
% Outputs:
%   dist : path from s to t
%   path : cell array with shortest path from s to t

% Initialize frontier set F and explored set E
F = [s];
E = [];

% Set initial cost to 0
v(s) = 0;

cnt = 0;
while ~isempty(F)
    
    % Search F backwards for the minimum value node
    lim = inf;
    for k = F(end:-1:1)
        val = v(k);
        if val < lim
            lim = val;
            i = k;
        end
    end
    
    % Extract vertex and add to explored set
    F(F == i) = [];
    cnt = cnt + 1;
    E = [E i];
    
    % If the vertex is the target, terminate
    if i == t
        break;
    end
    
    % Otherwise get vertex neighbors and update value
    [Ni,WNi] = neighbors(i);
    for k = 1:length(Ni);
        j = Ni{k};
        % Neighbor has not been visited
        if ~ismember(j,[F E])
            v(j) = v(i) + WNi(k);
            F = [F j];
            P(j) = i;
        % Neighbor is visited but might not have min value
        elseif ismember(j,F)
            [v(j),ind] = min([v(j),v(i) + WNi(k)]);
            if ind == 2
                P(j) = i;
            end
        % Otherwise the vertex must be in the explored set
        elseif v(j) > v(i) + WNi(k)
            v(j) = v(i) + WNi(k);
            P(j) = i;
            F = [F j];
            % E(E == j) = [];
        end
    end

end

% Construct path
path = {};
path{1} = t;
while P(path{end}) ~= 0
    path{end+1} = P(path{end});
end
path = path(end:-1:1);
dist = v(t);

% Report the number of set extractions
fprintf('Finished in %d set extractions\n',cnt);

end

