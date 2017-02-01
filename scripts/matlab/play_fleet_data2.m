path(path,'skyler')
bagfile = 'skyler/recent_tests/adaptive_plot_radius/success/minrad40_maxrad80_delay3.0_RETRY_delay2.5.bag';
% bagfile = 'skyler/recent_tests/protection_radius/failed/protection_radius_60_test5.bag';
data = processAllTopics(bagfile);

%sim_range = [0 0.9];
start_sim_time = 0.5;
stop_sim_time = 40.0;

plot_intruder       = 1;
plot_uav_goal       = 0;
plot_uav_actual     = 1;
plot_fleet_pose     = 0;
plot_waypoint_path  = 0;
plot_path_trajectory= 0;
plot_trajectory_endpoints = 0;
plot_uav_xyz        = 0;
plot_intruder_xyz   = 0;
plot_intruder_dots  = 1;
plot_actual_squares = 1;
plot_goal_squares   = 0;
square_cnt      = 10;



uav1_goal = [data.uav1_stamped_goal.pose.position; data.uav1_stamped_goal.time];
uav2_goal = [data.uav2_stamped_goal.pose.position; data.uav2_stamped_goal.time];
uav3_goal = [filedata.uav3_stamped_goal.pose.position; data.uav3_stamped_goal.time];
uav4_goal = [data.uav4_stamped_goal.pose.position; data.uav4_stamped_goal.time];

uav1_pose = [data.fleet.uav1.ground_truth.odometry.pose.position; data.fleet.uav1.ground_truth.odometry.time];
uav2_pose = [data.fleet.uav2.ground_truth.odometry.pose.position; data.fleet.uav2.ground_truth.odometry.time];
uav3_pose = [data.fleet.uav3.ground_truth.odometry.pose.position; data.fleet.uav3.ground_truth.odometry.time];
uav4_pose = [data.fleet.uav4.ground_truth.odometry.pose.position; data.fleet.uav4.ground_truth.odometry.time];

fleet_pose = [data.fleet_pose.pose.position; data.fleet_pose.time];

intruder_pose = [data.intruder.ground_truth.odometry.pose.position; data.intruder.ground_truth.odometry.time];

waypoints = [data.waypoint.pose; data.waypoint.time];

path_coeff = [data.path_coeff.x_path_coeff; data.path_coeff.y_path_coeff; data.path_coeff.z_path_coeff; data.path_coeff.time];

%stop_goal_plot = ceil(size(uav1_goal,2)*sim_range(2));
%stop_fleet_plot = ceil(size(fleet_pose,2)*sim_range(2));
%stop_pose_plot = ceil(size(uav1_pose,2)*sim_range(2));

% Trim to the specified time interval
uav1_goal = getSimInterval(uav1_goal, start_sim_time, stop_sim_time);
uav2_goal = getSimInterval(uav2_goal, start_sim_time, stop_sim_time);
uav3_goal = getSimInterval(uav3_goal, start_sim_time, stop_sim_time);
uav4_goal = getSimInterval(uav4_goal, start_sim_time, stop_sim_time);

uav1_pose = getSimInterval(uav1_pose, start_sim_time, stop_sim_time);
uav2_pose = getSimInterval(uav2_pose, start_sim_time, stop_sim_time);
uav3_pose = getSimInterval(uav3_pose, start_sim_time, stop_sim_time);
uav4_pose = getSimInterval(uav4_pose, start_sim_time, stop_sim_time);

fleet_pose = getSimInterval(fleet_pose, start_sim_time, stop_sim_time);
intruder_pose = getSimInterval(intruder_pose, start_sim_time, stop_sim_time);
waypoints = getSimInterval(waypoints, start_sim_time, stop_sim_time);
path_coeff = getSimInterval(path_coeff, start_sim_time, stop_sim_time);

figure(1);
clf(1);



hold('on');
axis('equal');
xlabel('X');
ylabel('Y');
zlabel('Z');

min_x = 0;
min_y = 0;
min_z = 0;
max_x = 5;
max_y = 5;
max_z = 5;


%axis([-50 100 -50 100 -10 100]);


% define persistent variables 
fleet_square_index = 1;
intruder_index = 1;

% first time function is called, initialize plot and persistent vars
figure(1), clf
hold on
rectangle('Position', [0 0 2 2], 'Curvature', 0);

% Fleet figure
vert1 = uav1_pose(1:3, fleet_square_index);
vert2 = uav2_pose(1:3, fleet_square_index);
vert3 = uav3_pose(1:3, fleet_square_index);
vert4 = uav4_pose(1:3, fleet_square_index);
vert5 = uav1_pose(1:3, fleet_square_index);
square = [vert1, vert2, vert3, vert4, vert5];
fleet_handle = plot3(square(1,:), square(2,:), -square(3,:), 'r');  

fleet_min_x = min(square(1,:));
fleet_max_x = max(square(1,:));
fleet_min_y = min(square(2,:));
fleet_max_y = max(square(2,:));
fleet_min_z = min(-1.0*square(3,:));
fleet_max_z = max(-1.0*square(3,:));

% Intruder figure
intruder_handle = plot3(intruder_pose(1,intruder_index), intruder_pose(2,intruder_index), -intruder_pose(3,intruder_index), '+k');            

intruder_min_x = min(intruder_pose(1,intruder_index));
intruder_max_x = max(intruder_pose(1,intruder_index));
intruder_min_y = min(intruder_pose(2,intruder_index));
intruder_max_y = max(intruder_pose(2,intruder_index));
intruder_min_z = min(-1.0*intruder_pose(3,intruder_index));
intruder_max_z = max(-1.0*intruder_pose(3,intruder_index));

axis([min_x-5 max_x+5 min_y-5 max_y+5 min_z-5 max_z+5]);  

tstart = tic;
% t = linspace(start_sim_time,stop_sim_time, 5000);
fig = uifigure('Position',[100 100 350 75]);
t_slider = uislider(fig,...
                    'Position', [50 50 250 5],...
                    'ValueChangingFcn', @(t_slider,event) timeChange(event));
                
                



% for i = 1:length(t)
% while (1)


    time_passed = 0;
    
    for i = 1:size(uav1_pose,2)
       
    end
    if( fleet_square_index < size(uav1_pose,2) && uav1_pose(4,fleet_square_index) <= telapsed )
%         disp('plot fleet');
        vert1 = uav1_pose(1:3, fleet_square_index);
        vert2 = uav2_pose(1:3, fleet_square_index);
        vert3 = uav3_pose(1:3, fleet_square_index);
        vert4 = uav4_pose(1:3, fleet_square_index);
        vert5 = uav1_pose(1:3, fleet_square_index);
        square = [vert1, vert2, vert3, vert4, vert5];
        
        fleet_min_x = min(square(1,:));
        fleet_max_x = max(square(1,:));
        fleet_min_y = min(square(2,:));
        fleet_max_y = max(square(2,:));
        fleet_min_z = min(-1.0*square(3,:));
        fleet_max_z = max(-1.0*square(3,:));

        set(fleet_handle, 'XData', square(1,:), 'YData', square(2,:), 'ZData', -square(3,:));
        drawnow
        fleet_square_index = fleet_square_index + 1;
        
    end
    
    if( intruder_index < size(intruder_pose,2) && intruder_pose(4,intruder_index) <= telapsed )
%         disp('plot intruder');
        intruder_min_x = min(intruder_pose(1,intruder_index));
        intruder_max_x = max(intruder_pose(1,intruder_index));
        intruder_min_y = min(intruder_pose(2,intruder_index));
        intruder_max_y = max(intruder_pose(2,intruder_index));
        intruder_min_z = min(-1.0*intruder_pose(3,intruder_index));
        intruder_max_z = max(-1.0*intruder_pose(3,intruder_index));

        set(intruder_handle, 'XData', intruder_pose(1,intruder_index), 'YData', intruder_pose(2,intruder_index), 'ZData', -intruder_pose(3,intruder_index));        
        drawnow
        intruder_index = intruder_index + 1;
    end
    
    min_x = min([min_x,fleet_min_x,intruder_min_x]);
    max_x = max([max_x,fleet_max_x,intruder_max_x]);
    min_y = min([min_y,fleet_min_y,intruder_min_y]);
    max_y = max([max_y,fleet_max_y,intruder_max_y]);
    min_z = min([min_z,fleet_min_z,intruder_min_z]);
    max_z = max([max_z,fleet_max_z,intruder_max_z]);
    axis([min_x-5 max_x+5 min_y-5 max_y+5 min_z-5 max_z+5]);    
    
end
    

if plot_goal_squares == true
    for i = 0:square_cnt
        size_difference_buffer = 5;
        square_step_size = idivide(uint16(size(uav1_goal,2))-size_difference_buffer,uint16(square_cnt));
        index = i*square_step_size + 1;
        vert1 = uav1_goal(1:3, index);
        vert2 = uav2_goal(1:3, index);
        vert3 = uav3_goal(1:3, index);
        vert4 = uav4_goal(1:3, index);
        vert5 = uav1_goal(1:3, index);
        square = [vert1, vert2, vert3, vert4, vert5];    


        plot3(square(1,:), square(2,:), -square(3,:), 'g')
    end    
end 

for i = 0:square_cnt    
    size_difference_buffer = 2;
    step_size = idivide(uint16(size(intruder_pose,2))-size_difference_buffer,uint16(square_cnt));
    index = i*step_size + 1;  
    
    if plot_intruder_dots == true
        plot3(intruder_pose(1,index), intruder_pose(2,index), -intruder_pose(3,index), '+k')
    end
    
end 

if plot_fleet_pose == true
    plot3(fleet_pose(1,:),fleet_pose(2,:), -1.0*fleet_pose(3, :), '-m');
end

if plot_intruder == true
    plot3(intruder_pose(1,:),intruder_pose(2,:), -1.0*intruder_pose(3, :), 'k');
end

if plot_uav_goal == true
    plot3(uav1_goal(1,:),uav1_goal(2,:), -1.0*uav1_goal(3, :), '-.r');
    plot3(uav2_goal(1,:),uav2_goal(2,:), -1.0*uav2_goal(3, :), '-.g');
    plot3(uav3_goal(1,:),uav3_goal(2,:), -1.0*uav3_goal(3, :), '-.b');
    plot3(uav4_goal(1,:),uav4_goal(2,:), -1.0*uav4_goal(3, :), '-.c');
end

if plot_uav_actual == true
    plot3(uav1_pose(1,:),uav1_pose(2,:), -1.0*uav1_pose(3, :), '-r');
    plot3(uav2_pose(1,:),uav2_pose(2,:), -1.0*uav2_pose(3, :), '-g');
    plot3(uav3_pose(1,:),uav3_pose(2,:), -1.0*uav3_pose(3, :), '-b');
    plot3(uav4_pose(1,:),uav4_pose(2,:), -1.0*uav4_pose(3, :), '-c');
end

if plot_waypoint_path == true
   plot3(waypoints(1,:), waypoints(2,:), -1.0*waypoints(3,:), 'm--o');
end

color_step = 1;
%color = 1;
RGB = [0 0 0];
endpoints = zeros(3,square_cnt + 1);
endpoint_index = 1;
   
for i = 0:square_cnt
    tau = linspace(0, 1, 100).'; 
    square_step_size = idivide(uint16(size(path_coeff,2))-size_difference_buffer,uint16(square_cnt));
    index = i*square_step_size + 1;

    x_coeff = path_coeff(1:4,index);
    y_coeff = path_coeff(5:8,index);
    z_coeff = path_coeff(9:12,index);
    Phi = [ones(size(tau,1),1), tau, tau.^2, tau.^3];
    x = Phi*x_coeff;
    y = Phi*y_coeff;
    z = Phi*z_coeff;

    RGB(1) = mod(i,2);
    RGB(2) = mod(int8(i/2),2);
    RGB(3) = mod(int8(i/4),2);

    %RGB(color) = RGB(color) + color_step;
    %if(RGB(color) >= 1)
    %   RGB(color) = 0; 
    %end
    %plot3(x,y,-z,'-','Color',RGB);
    if plot_path_trajectory == true
%         plot3(x,y,-z,'-r');
        plot3(x,y,-z,'-','Color',RGB);
    end
    endpoints(:,endpoint_index) = [x(length(x)); y(length(y));-z(length(z))];
    endpoint_index = endpoint_index + 1;
end

if plot_trajectory_endpoints == true
   plot3(endpoints(1,:), endpoints(2,:), endpoints(3,:), '-+b');
end

if plot_uav_xyz == true
    figure(2);
    clf(2);
    x_plot = subplot(3,1,1);
    y_plot = subplot(3,1,2);
    z_plot = subplot(3,1,3);
    
    hold('on');
    
    if plot_intruder_xyz == true 
        plot(x_plot, intruder_pose(4,:), intruder_pose(1,:), 'k', uav1_goal(4,:), uav1_goal(1,:), '-g', uav1_pose(4,:), uav1_pose(1,:), '-r')
        plot(y_plot, intruder_pose(4,:), intruder_pose(2,:), 'k', uav1_goal(4,:), uav1_goal(2,:), '-g', uav1_pose(4,:), uav1_pose(2,:), '-r')
        plot(z_plot, intruder_pose(4,:), -1.0*intruder_pose(3,:), 'k', uav1_goal(4,:), -1.0*uav1_goal(3,:), '-g', uav1_pose(4,:), -1.0*uav1_pose(3,:), '-r')
        
    else
        plot(x_plot, uav1_goal(4,:), uav1_goal(1,:), '-g', uav1_pose(4,:), uav1_pose(1,:), '-r') 
        plot(y_plot, uav1_goal(4,:), uav1_goal(2,:), '-g', uav1_pose(4,:), uav1_pose(2,:), '-r') 
        plot(z_plot, uav1_goal(4,:), -1.0*uav1_goal(3,:), '-g', uav1_pose(4,:), -1.0*uav1_pose(3,:), '-r')
    end
    title(x_plot, 'X goal vs. pose')
    title(y_plot, 'Y goal vs. pose')
    title(z_plot, 'Z goal vs. pose')
end



function time = timeChange(event)
    time = event.Value
end