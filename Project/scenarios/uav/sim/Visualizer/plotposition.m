function [] = plotposition(t, poses)
    
    a = struct2cell(poses);
    b = [a{1,:}];
    plot(t,b);
    
    legend('x','y','z');
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('World Frame Position');
    grid on;
end