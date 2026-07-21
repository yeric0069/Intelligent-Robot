function result_plot(xresult, uresult)
    x_pos = xresult(1,:);
    y_pos = xresult(2,:);
    heading = xresult(3,:);
    vol = xresult(4,:);
    acc = uresult(2,:);
    
    t_end = length(x_pos);
    time = 1:t_end;
    
    % plot
    figure;
    hold on;
    plot(time, x_pos, 'r-');
    plot(time, y_pos, 'b-');
    xlabel('Time (s)'); 
    ylabel('Position (m)'); 
    legend('X', 'Y'); 
    
    figure, hold
    plot(time, heading*180/pi)
    xlabel('Time (s)'); 
    ylabel('Vehicle Heading, deg')

    figure, hold
    plot(time, vol)
    xlabel('Time (s)'); 
    ylabel('Forward Speed, m/s')
    
    figure, hold
    plot(time, acc, 'r')
    xlabel('Time (s)'); 
    ylabel('Acceleration, m/s^2')
    grid on;
end