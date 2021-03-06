function showPath2(path,A)
    initializeGraphics(A);
    global handles;
        for i=1:size(path,2)
            v = path{i};
            changeColor(v(end:-1:1),'r');
            pause(0.03);
        end
end

function initializeGraphics(A)
    figure;
    hold on;
    set(gca,'FontSize',18);
    xlabel('Time step');
    ylabel('Stock amount');
    N=size(A,2);
    M=size(A,1);
    global handles
    handles = cell(N,M);
    axis([-0.2,N+0.2,-0.2,M+0.2]);
    hold on
    daspect([1,1,1]);
        for i=1:N
            for j=1:M
                switch A(j,i)
                    case 0,
                        col = 'k';
                    case 1,
                        col = 'w';
                    case 2,
                        col = 'b';
                    case 3,
                        col = 'g';
                    case 4,
                        col = 'y';
                end
                handles{i,j}=rectangle('Position',...
                    [i-1,j-1,1,1],'LineWidth',1,'FaceColor',col);
            end
        end
    set(gca,'XTick',[],'YTick',[]);
end

function changeColor(x,col)
    global handles;
    delete(handles{x(1),x(2)});
    handles{x(1),x(2)}=rectangle('Position',...
        [x(1)-1,x(2)-1,1,1],'LineWidth',1,'FaceColor',col);
    drawnow;
end

