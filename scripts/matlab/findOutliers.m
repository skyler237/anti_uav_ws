function [outliers, success_outliers, failed_inliers, possible_failed_mistakes, possible_success_mistakes]...
                = findOutliers( results )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    N = size(results,2);
    test_numbers =  results(1,:);
    successes   =   results(2,:);
    points      =   results(3:4,:);
    positions   =   results(6:8,:);
    distances   =   results(9,:);
    times       =   results(10,:);


    net_width = 6;
    net_edge = net_width/2;
    general_outlier_margin = net_width;
    success_outlier_margin = net_edge*1.05;
    failed_inliers_margin = net_edge*0.95;
    possible_failed_mistakes_margin = 50;
    possible_success_mistakes_margin = 5;
    
    outliers = [];
    success_outliers = [];
    failed_inliers = [];
    possible_failed_mistakes = [];
    possible_success_mistakes = [];
    
    % Find distant outliers
    for i=1:N
        if (successes(i))
            if (abs(points(1,i)) > possible_success_mistakes_margin || abs(points(2,i)) > possible_success_mistakes_margin)
                possible_success_mistakes = [possible_success_mistakes, [test_numbers(i); points(1:2,i)]];            
            elseif (abs(points(1,i)) > success_outlier_margin || abs(points(2,i)) > success_outlier_margin)
                success_outliers = [success_outliers, [test_numbers(i); points(1:2,i)]];
            end
        else
            if (abs(points(1,i)) > possible_failed_mistakes_margin || abs(points(2,i)) > possible_failed_mistakes_margin)
                possible_failed_mistakes = [possible_failed_mistakes, [test_numbers(i); points(1:2,i)]];
            elseif (abs(points(1,i)) < failed_inliers_margin && abs(points(2,i)) < failed_inliers_margin)
                failed_inliers = [failed_inliers, [test_numbers(i); points(1:2,i)]];
            elseif (abs(points(1,i)) > general_outlier_margin || abs(points(2,i)) > general_outlier_margin)
                outliers = [outliers, [test_numbers(i); points(1:2,i)]];
            end
        end        
    end
    
    

    fprintf('General Outliers: (more than %f m beyond net edge)\n', general_outlier_margin - net_edge);
    fprintf('       Point #:   X:       Y:\n');
    format shortG
    disp(outliers');
    
    fprintf('\n');
    
    fprintf('Success Outliers: (more than %f m beyond net edge)\n', success_outlier_margin - net_edge);
    fprintf('       Point #:   X:       Y:\n');    
    disp(success_outliers');
    
    fprintf('\n');
    
    fprintf('Failed Inliers: (more than %f m within net edge)\n', net_edge - failed_inliers_margin);
    fprintf('       Point #:   X:       Y:\n');
    disp(failed_inliers');
    
    fprintf('\n');
    
    fprintf('Possible Failed Mistakes: (more than %f m beyond net edge)\n', possible_failed_mistakes_margin - net_edge);
    fprintf('       Point #:   X:       Y:\n');
    disp(possible_failed_mistakes');
       
    
    fprintf('\n');
    
    fprintf('Possible Success Mistakes: (more than %f m beyond net edge)\n', possible_success_mistakes_margin - net_edge);
    fprintf('  Point #:   X:       Y:\n');
    disp(possible_success_mistakes');
       
end

