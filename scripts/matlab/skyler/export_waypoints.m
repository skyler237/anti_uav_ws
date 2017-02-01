waypoint_filename = '/home/skyler/waypoints.csv';

waypoint_data = zeros(size(data.waypoint.pose,2)+1,size(data.waypoint.pose,1));
waypoint_data(1:size(waypoint_data,1)-1, :) = data.waypoint.pose';
waypoint_data(size(waypoint_data,1), 3) = -10;
csvwrite(waypoint_filename,waypoint_data)