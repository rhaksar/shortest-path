function [ Nbors ] = maze8_observer( s,A )

[Nr,Nc] = size(A);
[r,c] = ind2sub([Nr,Nc],s);

Nbors = [];

thresh = 2;

if r+1 <= Nr && A(r+1,c) >= thresh
    Nbors = [Nbors sub2ind([Nr,Nc],r+1,c)];
end
if r-1 >= 1 && A(r-1,c) >= thresh
    Nbors = [Nbors sub2ind([Nr,Nc],r-1,c)];
end

if c+1 <= Nc && A(r,c+1) >= thresh
    Nbors = [Nbors sub2ind([Nr,Nc],r,c+1)];
end
if c-1 >= 1 && A(r,c-1) >= thresh
    Nbors = [Nbors sub2ind([Nr,Nc],r,c-1)];
end

if r+1 <= Nr && c+1 <= Nc && A(r+1,c+1) >= thresh
    Nbors = [Nbors sub2ind([Nr,Nc],r+1,c+1)];
end
if r-1 >= 1 && c-1 >= 1 && A(r-1,c-1) >= thresh
    Nbors = [Nbors sub2ind([Nr,Nc],r-1,c-1)];
end

if r+1 <= Nr && c-1 >= 1 && A(r+1,c-1) >= thresh
    Nbors = [Nbors sub2ind([Nr,Nc],r+1,c-1)];
end
if r-1 >= 1 && c+1 <= Nc && A(r-1,c+1) >= thresh
    Nbors = [Nbors sub2ind([Nr,Nc],r-1,c+1)];
end




end

