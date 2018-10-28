function [] = plotaccelerations(t, ctrl, twists)

    hold on
%     % desired accleration
    controlCells = struct2cell(ctrl);
    desiredAccelerations = [controlCells{3,:}];
    plot(t,desiredAccelerations(1,:),'--r');
    plot(t,desiredAccelerations(2,:),'--g');
    plot(t,desiredAccelerations(3,:)-9.81,'--b');

    
    % actual acceleration (world frame conversion)
    temp = struct2cell(twists);
    worldAccelerations = [temp{2,:}];
    
    plot(t,worldAccelerations(1,:),'r');
    plot(t,worldAccelerations(2,:),'g');
    plot(t,worldAccelerations(3,:),'b');
    
    legend('accXDes','accYDes','accZDes','accX','accY','accZ');
    xlabel('Time (s)');
    ylabel('Accelerations (m)');
    title('Desired vs. Actual Acceleration');
    grid on;
end