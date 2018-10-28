function[] = plot_quaternions(ttotal,a1,b1,a2,b2)

disprate = 30; %Hz
t = ttotal;
disprate_idx = round((size(t,1)/(t(end)-t(1)))/disprate);


for i = 1:disprate_idx:size(t,1)

    plot_arrow(0,0,a1(i),b1(i));
    hold on
    plot_arrow(0,0,a2(i),b2(i));
    axis([-1 1 -1 1]);
    grid on;
    hold off
    drawnow;
    pause(0.05)
end

