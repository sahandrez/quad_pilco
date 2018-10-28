function [] = plotcontrolrpm(t, ctrl)

    a = struct2cell(ctrl);
    b = [a{6,:}];
    plot(t,abs(b));
%     (sqrt(abs(b))-1000)*850/6000+1075
    legend('Motor 1','Motor 2','Motor 3','Motor 4');
    xlabel('Time (s)');
    ylabel('Motor (RPM)');
    title('Control Output Motor Speeds');
    grid on;
    
end