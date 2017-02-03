function [ removed_tests ] = removeFaultyTests(test_path, possible_failed_mistakes, possible_success_mistakes )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% test_path = '~/counter_uas/scripts/matlab/skyler/recent_tests/analysis/autotest/test16/test/';

removed_tests = [];


% Handle failed tests
for i=1:size(possible_failed_mistakes,2)
    test_number = num2str(possible_failed_mistakes(1,i));
    bagfile = strcat(test_path,'auto_test', test_number, '.bag');
    results_file = strcat(test_path,'results', test_number, '.bag');
    
    try
        [T, data] = evalc('processAllTopics(bagfile);');
    catch
        disp(['Removing ' bagfile]);
        if(exist(bagfile,'file'))
            system(['rm ' bagfile ' ' results_file]);
            removed_tests = [removed_tests, test_number];
        end        
        continue;
    end
    
    % Check to see if one of the UAV's is "stuck"
    uav1_pose = [data.fleet.uav1.ground_truth.odometry.pose.position];
    uav2_pose = [data.fleet.uav2.ground_truth.odometry.pose.position];
    uav3_pose = [data.fleet.uav3.ground_truth.odometry.pose.position];
    uav4_pose = [data.fleet.uav4.ground_truth.odometry.pose.position];
    intruder_pose = [data.intruder.ground_truth.odometry.pose.position];
    
    x_ranges = [range(uav1_pose(1,:)), range(uav2_pose(1,:)), range(uav3_pose(1,:)), range(uav4_pose(1,:))];
    
    %  vvv Check for UAV crash   vvv Check for no data   vvv Check for intruder crash
    if(range(x_ranges) > 30 || size(uav1_pose,2) == 0 || range(intruder_pose(2,:)) == 0)
       disp(['Removing ' bagfile]);
        if(exist(bagfile,'file'))
            system(['rm ' bagfile ' ' results_file]);
            removed_tests = [removed_tests, test_number];
        end 
    else
        if(exist(bagfile,'file'))        
            disp(['NOT removing ' bagfile]);
        end
    end

end

% Handle successful tests
for i=1:size(possible_success_mistakes,2)
    test_number = num2str(possible_success_mistakes(1,i));
    bagfile = strcat(test_path,'auto_test', test_number, '.bag');
    results_file = strcat(test_path,'results', test_number, '.bag');
    
    try
        [T, data] = evalc('processAllTopics(bagfile);');
    catch
        disp(['Removing ' bagfile]);
        if(exist(bagfile,'file'))
            system(['rm ' bagfile ' ' results_file]);
            removed_tests = [removed_tests, test_number];
        end
        continue;
    end
    
    % Check to see if one of the UAV's is "stuck"
    uav1_pose = [data.fleet.uav1.ground_truth.odometry.pose.position];
    uav2_pose = [data.fleet.uav2.ground_truth.odometry.pose.position];
    uav3_pose = [data.fleet.uav3.ground_truth.odometry.pose.position];
    uav4_pose = [data.fleet.uav4.ground_truth.odometry.pose.position];
    intruder_pose = [data.intruder.ground_truth.odometry.pose.position];
    
    x_ranges = [range(uav1_pose(1,:)), range(uav2_pose(1,:)), range(uav3_pose(1,:)), range(uav4_pose(1,:))];
    
    %  vvv Check for UAV crash   vvv Check for no data   vvv Check for intruder crash
    if(range(x_ranges) > 30 || size(uav1_pose,2) == 0 || range(intruder_pose(2,:)) == 0)
       disp(['Removing ' bagfile]);
        if(exist(bagfile,'file'))
            system(['rm ' bagfile ' ' results_file]);
            removed_tests = [removed_tests, test_number];
        end
    else
        if(exist(bagfile,'file'))        
            disp(['NOT removing ' bagfile]);
        end
    end
    
    removed_tests = sort(removed_tests);
end

