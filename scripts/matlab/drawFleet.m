function drawFleet(square, t)

    % define persistent variables 
    persistent link_handle
    persistent first
    
    % first time function is called, initialize plot and persistent vars
    if t < 1
        figure(1), clf
        hold on
        link_handle = plot3(square(1,:), square(2,:), -square(3,:), 'r');            
        axis([-80, 80, -80, -80, -5, 50]);        
    % at every other time step, redraw base and rod
    else 
        set(link_handle, 'XData', square(1,:), 'YData', square(2,:), 'ZData', -square(3,:));
        drawnow
    end
end

 