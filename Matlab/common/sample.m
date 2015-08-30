function [x] = sample(p)
    x = find(rand() < cumsum(p) , 1);
end
