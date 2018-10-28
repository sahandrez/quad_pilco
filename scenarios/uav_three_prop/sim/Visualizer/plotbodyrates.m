function [] = plotbodyrates(t, ctrl, twists)

    hold on
    % desired
    a = struct2cell(ctrl);
    b = [a{2,:}];
    c = [b.angVel];
    plot(t,c(1,:),'--r');
    plot(t,c(2,:),'--g');
    plot(t,c(3,:),'--b');
    
    %actual
    d = struct2cell(twists);
    e = [d{4,:}];
    plot(t,e(1,:),'r');
    plot(t,e(2,:),'g');
    plot(t,e(3,:),'b');
    
    legend('pDes','qDes','rDes','p','q','r');
    xlabel('Time (s)');
    ylabel('Body rate (rad/s)');
    title('Desired vs. Actual Body Rates');
    grid on;
end