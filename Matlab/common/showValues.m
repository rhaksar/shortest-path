function [v_mat] = showValues( v,A )
[Nr,Nc] = size(A);

v_mat = zeros(Nc,Nr);
for i = 1:length(v(1,:))
    [r,c] = ind2sub(size(A),v(1,i));
    v_mat(Nc-c+1,r) = v(2,i);
end

figure;
hold on;
h = bar3(flipud(v_mat));
for i = 1:length(h)
     zdata = get(h(i),'Zdata');
     set(h(i),'Cdata',zdata)
end
axis([0 Nr+1 0 Nc+1]);

end

