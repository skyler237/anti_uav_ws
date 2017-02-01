function [isIntercept, passed_fleet, int_point, int_position] = testIntercept(UAV_points, target_points, net_width)

    persistent prev_not_passed

    if(prev_not_passed == 0)
       prev_not_passed = 'true'; 
    end
    
    int_position = [0; 0; 0];
    
    % Points of each UAV
    p1 = UAV_points(:,1);
    p2 = UAV_points(:,2);
    p3 = UAV_points(:,3);
    p4 = UAV_points(:,4);

    % Averaged center of fleet
    p_center = mean(UAV_points,2);

    % Vectors from center to corner
    l = UAV_points - [p_center, p_center, p_center, p_center];

    % Normal vectors
    n1 = cross(l(:,2),l(:,1));
    n2 = cross(l(:,3),l(:,2));
    n3 = cross(l(:,4),l(:,3));
    n4 = cross(l(:,1),l(:,4));

    % Averaged normal vector (in case fleet is not all in same plane)
    n_avg = mean([n1, n2, n3, n4],2);
    n_avg = n_avg/norm(n_avg);
    normal = [p_center,(p_center + n_avg)];

    % Define body frame x_axis
    x_axis = (mean([p1,p2],2) - p_center);
    x_axis = x_axis/norm(x_axis);
    x_axis_line = [p_center, (p_center + x_axis)];

    % Body Frame y_axis
    y_axis = cross(x_axis, n_avg);
    y_axis_line = [p_center, (p_center + y_axis)];

    % ============= POINT 1 ================
    % Project target position onto fleet plane
    v1 = (target_points(:,1) - p_center);
    v_par1 = dot(v1,n_avg)*n_avg;
    v_perp1 = v1 - v_par1;
    proj_point1 = p_center + v_perp1;

    % Perpendicular distance of the target from the fleet plane
    distance1 = norm(v_par1);
    
    % X and Y coordinates of the target projection in fleet body frame 
        % (Relative to center)
    proj_x1 = dot(v_perp1, x_axis);
    proj_y1 = dot(v_perp1, y_axis);

    % ============= POINT 2 ================
    % Project target position onto fleet plane
    v2 = (target_points(:,2) - p_center);
    v_par2 = dot(v2,n_avg)*n_avg;
    v_perp2 = v2 - v_par2;
    proj_point2 = p_center + v_perp2;

    % Perpendicular distance of the target from the fleet plane
    distance2 = norm(v_par2);
    
    % X and Y coordinates of the target projection in fleet body frame 
        % (Relative to center)
    proj_x2 = dot(v_perp2, x_axis);
    proj_y2 = dot(v_perp2, y_axis);
    
    
    int_point = [(proj_x1 + proj_x2)/2;
                 (proj_y1 + proj_y2)/2];
    
    % Check to see if the fleet is between the two target points     
    % (potential intercept)    
    target_dist1 = norm(target_points(:,1));
    fleet_dist = norm(p_center);
    target_dist2 = norm(target_points(:,2)); 
    
    not_passed = 0;
    if(target_dist1 > fleet_dist && target_dist2 > fleet_dist)
       not_passed = 'true'; 
    elseif(target_dist1 < fleet_dist && target_dist2 < fleet_dist)
       not_passed = 'false';
    end
    
%     disp('================') 
%     disp('distance1')
%     disp(distance1)
%     disp('distance2')
%     disp(distance2)
%     disp('target_dist1')
%     disp(target_dist1)
%     disp('fleet_dist')
%     disp(fleet_dist)
%     disp('target_dist2')
%     disp(target_dist2)
    
    
    
    if (target_dist1 > fleet_dist && fleet_dist > target_dist2 && distance1 < 1 || ...
            (strcmp(prev_not_passed,'true') && strcmp(not_passed,'false')))
        passed_fleet = 1;
        int_position = p_center;
        
        % Check to see if the targets projected point is within the net
        pos_x_dist = norm(p_center - mean([p1,p4],2));
        pos_y_dist = norm(p_center - mean([p2,p1],2));
        neg_x_dist = norm(p_center - mean([p3,p2],2));
        neg_y_dist = norm(p_center - mean([p4,p3],2));
        
        if (int_point(1) > -neg_x_dist && int_point(1) < pos_x_dist &&...
                int_point(2) > -neg_y_dist && int_point(2) < pos_y_dist)
           isIntercept = 1;
           UAV_points = UAV_points;
           target_points = target_points;
           target_dist1 = target_dist1;
           fleet_dist = fleet_dist;
           target_dist2 = target_dist2;
        else           
           isIntercept = 0;
        end
    else
        passed_fleet = 0;
        isIntercept = 0;
    end
    
    if(target_dist1 > fleet_dist && target_dist2 > fleet_dist)
       prev_not_passed = not_passed; 
    elseif(target_dist1 < fleet_dist && target_dist2 < fleet_dist)
       prev_not_passed = not_passed;
    end

             
    % Plots
    if(passed_fleet)
%         square = [p1, p2, p3, p4, p1];
%         figure(3), clf;
%         hold on
%         axis equal
%         plot3(square(1,:), square(2,:), -square(3,:), 'r')
%         plot3(normal(1,:), normal(2,:), -normal(3,:),'b')
%         plot3(x_axis_line(1,:), x_axis_line(2,:), -x_axis_line(3,:),'k')
%         plot3(y_axis_line(1,:), y_axis_line(2,:), -y_axis_line(3,:),'g')
%         plot3(target_points(1,:), target_points(2,:), -target_points(3,:),'k+')
%         
%         if(isIntercept)
%             plot3(proj_point1(1), proj_point1(2), -proj_point1(3),'g+')
%         else
%             plot3(proj_point1(1), proj_point1(2), -proj_point1(3),'r+')
%         end
% 
%         figure(4), clf;
%         hold on
%         axis equal
%         rectangle('Position', [-net_width/2 -net_width/2 net_width, net_width])
%         
%         if(isIntercept)
%             plot(int_point(1), int_point(2),'g+');
%         else
%             plot(int_point(1), int_point(2),'r+');
%         end
%         
    end
end


