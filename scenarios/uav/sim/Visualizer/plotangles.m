function [] = plotangles(t, pose)

    a = struct2cell(pose);
    b = [a{3,:}];
    plot(t,b); 
    
    legend('roll','pitch','yaw');
    xlabel('Time (s)');
    ylabel('Euler Angles (rad)');
    title('Euler Angle Attitude');
    grid on;
end