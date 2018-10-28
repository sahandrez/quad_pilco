function [] = ploterrorquaternion(t, ctrl)

    hold on
    % desired
    a = struct2cell(ctrl);
    b = [a{4,:}];
    x = b(2,:);
    y = b(3,:);
    plot(t,x, '--r', 'LineWidth', 2);
    plot(t,y, '--b', 'LineWidth', 2);

    legend('x','y');
    xlabel('Time (s)');
    ylabel('Unit Quaternion Values');
    title('Error Quaternion');
    grid on;