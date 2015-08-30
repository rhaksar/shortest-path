function [ Nbors ] = stock_obs( x,A,p )

[Nr,Nc] = size(A);
[r,c] = ind2sub([Nr,Nc],x);

Nbors = [];

t = 1;
tst = sample([p,1-p]);

if r-1 >= 1 && c+1 <= Nc && A(r-1,c+1) >= t && c ~= Nc-1 && tst == 1
    Nbors = [Nbors sub2ind(size(A),r-1,c+1)];
end


end

