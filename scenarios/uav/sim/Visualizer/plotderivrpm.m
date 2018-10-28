function [] = plotderivrpm(t, ctrl)

    a = struct2cell(ctrl);
%     b = [a{2,:}];
%     plot(t,abs(b));
%     hold on;
    b = [a{1,:}];
    plot(t,abs(b));
    legend('Motor 1','Motor 2','Motor 3','Motor 4');
    xlabel('Time (s)');
    ylabel('Motor (RPM Accelerations)');
    title('Control Output Motor Speeds');
    grid on;
    
end