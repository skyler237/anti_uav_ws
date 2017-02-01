% p = [4, 3, 6];
% O = [0, 0, 0];
% x_axis = [1, 1, -1];
% n = [0, 1, 1];
% 
% x = dot((p - O),x_axis)
% y = dot((p-O),(cross(n,x_axis)))

net_width = 6.0;

% Points of each UAV
p1 = [77.7135, 1.0419, -13.8197];
p2 = [76.5299, 1.0653, -19.9495];
p3 = [76.5235, -4.9387, -20.0724];
p4 = [77.7430, -4.9665, -14.0299];
p = [p1; p2; p3; p4];

% Averaged center of fleet
pf = mean(p);

% Vectors from center to corner
l = p - [pf; pf; pf; pf];

% Normal vectors
n1 = cross(l(2,:),l(1,:));
n2 = cross(l(3,:),l(2,:));
n3 = cross(l(4,:),l(3,:));
n4 = cross(l(1,:),l(4,:));

% Averaged normal vector (in case fleet is not all in same plane)
n_avg = (mean([n1; n2; n3; n4]));
n_avg = n_avg/norm(n_avg);
normal = [pf',(pf + n_avg)'];

% Define body frame x_axis
x_axis = (mean([p1;p4]) - pf);
x_axis = x_axis/norm(x_axis);
x_axis_line = [pf', (pf + x_axis)'];

% Body Frame y_axis
y_axis = cross(x_axis, n_avg);
y_axis_line = [pf', (pf + y_axis)'];

% Position of the target
p_target = [78, -3, -18];

% Project target position onto fleet plane
v = (p_target - pf);
v_par = dot(v,n_avg)*n_avg;
v_perp = v - v_par;
proj_target = pf + v_perp;

% Perpendicular distance of the target from the fleet plane
distance = norm(v_par);

% X and Y coordinates of the target projection in fleet body frame 
    % (Relative to center)
proj_x = dot(v_perp, x_axis);
proj_y = dot(v_perp, y_axis);

% Plots
square = [p1', p2', p3', p4', p1'];
figure(1), clf;
hold on
axis equal
plot3(square(1,:), square(2,:), -square(3,:), 'r')
plot3(normal(1,:), normal(2,:), -normal(3,:),'b')
plot3(x_axis_line(1,:), x_axis_line(2,:), -x_axis_line(3,:),'k')
plot3(y_axis_line(1,:), y_axis_line(2,:), -y_axis_line(3,:),'g')
plot3(p_target(1), p_target(2), -p_target(3),'k+')
plot3(proj_target(1), proj_target(2), -proj_target(3),'m+')


% Check to see if the targets projected point is within the net
pos_x_dist = norm(pf - mean([p1;p4]));
pos_y_dist = norm(pf - mean([p2;p1]));
neg_x_dist = norm(pf - mean([p3;p2]));
neg_y_dist = norm(pf - mean([p4;p3]));

if (proj_x > -neg_x_dist && proj_x < pos_x_dist &&...
        proj_y > -neg_y_dist && proj_y < pos_y_dist)
   isInside = 1 
else
   isInside = 0
end


