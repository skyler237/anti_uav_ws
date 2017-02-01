function [ isSuccess, intercept_point, intercept_position, time ] = findIntercept( rosbag )
%FIND_INTERCEPT
% Checks if there is an intercept within the given rosbag
  
    bagfile = rosbag;
    data = processAllTopics(bagfile);
    
    net_width = 6.0;
    isSuccess = 0;
    time = 0;

    uav1_pose = [data.fleet.uav1.ground_truth.odometry.pose.position; data.fleet.uav1.ground_truth.odometry.time];
    uav2_pose = [data.fleet.uav2.ground_truth.odometry.pose.position; data.fleet.uav2.ground_truth.odometry.time];
    uav3_pose = [data.fleet.uav3.ground_truth.odometry.pose.position; data.fleet.uav3.ground_truth.odometry.time];
    uav4_pose = [data.fleet.uav4.ground_truth.odometry.pose.position; data.fleet.uav4.ground_truth.odometry.time];

    intruder_pose = [data.intruder.ground_truth.odometry.pose.position; data.intruder.ground_truth.odometry.time];
    
    uav1_index = 1;
    uav2_index = 1;
    uav3_index = 1;
    uav4_index = 1;
    for i=1:size(intruder_pose,2)-1
        % Find the appropriate UAV positions for the given intruder pose
        while(uav1_index < size(uav1_pose,2) && uav1_pose(4,uav1_index) < intruder_pose(4,i))
            uav1_index = uav1_index + 1;
        end
        
        while(uav2_index < size(uav2_pose,2) && uav2_pose(4,uav2_index) < intruder_pose(4,i))
            uav2_index = uav2_index + 1;
        end

        while(uav3_index < size(uav3_pose,2) && uav3_pose(4,uav3_index) < intruder_pose(4,i))
            uav3_index = uav3_index + 1;
        end

        while(uav4_index < size(uav4_pose,2) && uav4_pose(4,uav4_index) < intruder_pose(4,i))
            uav4_index = uav4_index + 1;
        end
        
        
        time = intruder_pose(4,i);
        UAV_points = [uav1_pose(1:3,uav1_index), uav2_pose(1:3,uav2_index), uav3_pose(1:3,uav3_index), uav4_pose(1:3,uav4_index)];
        target_points = [intruder_pose(1:3,i), intruder_pose(1:3,i+1)];
        [isIntercept, passed_fleet, intercept_point, intercept_position] = testIntercept(UAV_points, target_points, net_width);

        if(isIntercept)
           isSuccess = 1;
           return;
        elseif (passed_fleet)
           return;
        else
           time = 0;
        end
    end
end

