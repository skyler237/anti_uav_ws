path(path,'skyler')
clear all

% test_path = 'skyler/recent_tests/analysis/autotest/test17/';
test_path = 'skyler/recent_tests/analysis/pronav/test9/';
% test_path = 'skyler/recent_tests/analysis/smooth_prediction_path/test4/';

results_file = strcat(test_path,'results.csv');
old_results = [];
if(exist(results_file, 'file'))
    old_results = csvread(results_file);
end

FileList = dir(strcat(test_path,'results*.bag'));
N = size(FileList,1);

net_width = 6;

times = [];
points = [];
positions = [];
distances = [];
successes = [];
test_numbers = [];

figure(1), clf
hold on
axis equal
rectangle('Position', [-net_width/2 -net_width/2 net_width, net_width])

for k=1:N
   bagfile = FileList(k).name;
   
   strings = split(bagfile, {'results','.bag'});
   test_number = str2double(strings(2));
   
   % Check if the bag has been processed before
   if(isempty(old_results))
      old_test_index = [];
   else
      old_test_index = find(old_results(1,:) == test_number, 1);
   end  

   if(~isempty(old_test_index))
       isSuccess = old_results(2, old_test_index);
       int_point = old_results(3:5, old_test_index);
       int_position = old_results(6:8, old_test_index);
       distance = old_results(9, old_test_index);
       time = old_results(10, old_test_index);   
   else
       % Bag has not yet been processed
       fprintf('Processing file: %s\n\r', strcat(test_path,bagfile));
   
       try
        data = processAllTopics(strcat(test_path,bagfile));
        data = data.results;
       catch
        continue
       end    
       
       isSuccess = data.isIntercept;
       int_point = [data.intercept_point];
       int_position = [data.intercept_pose];
       distance = data.intercept_radius;
       time = data.intercept_time;
   end   
   
   
   if(isSuccess)
       plot(int_point(1), int_point(2),'g+')
   else
       plot(int_point(1), int_point(2),'r+')
   end
   
   test_numbers = [test_numbers, test_number];
   successes = [successes,isSuccess];
   points = [points, int_point];
   positions = [positions, int_position];
   distances = [distances, distance];
   times = [times,time];
   
end

hold off

fprintf('============ RESULTS =============\n');
fprintf('Out of %d simulations, %d were successful\n', N, sum(successes));
fprintf('Success rate = %f\n\n', sum(successes)/N);

fprintf('Average point = (%f, %f)\n', mean(points(1,:)), mean(points(2,:)));
fprintf('Average intercept time = %f\n', mean(times));
fprintf('Average intercept radius = %f\n\n', mean(distances));

results = [test_numbers;successes;points;positions;distances;times];
results = sortrows(results')';

% new_results =  results(:,...
%     find([old_results(1,:), zeros(1, size(results,2) - size(old_results,2))] ~= results(1,:), 1):size(results,2));
       
new_results = [];
if(~isempty(old_results))
    for i=1:size(results,2)
        if(isempty(find(old_results(1,:) == results(1,i),1)))
            new_results = [new_results, results(:,i)];
        end
    end
else
    new_results = results;
end



format compact
% Check if there are any new results
if(size(new_results,2) ~= 0)
    [T, outliers, success_outliers, failed_inliers, ...                
        possible_failed_mistakes, possible_success_mistakes] = evalc('findOutliers(new_results);');
%         possible_failed_mistakes, possible_success_mistakes] = findOutliers(results);
    
    
    removed_tests = [removeFaultyTests(test_path, possible_failed_mistakes, possible_success_mistakes),...
                    removeFaultyTests(test_path, outliers, success_outliers),...
                    removeFaultyTests(test_path, failed_inliers, [])];
                
%     result_index = 1;
%     test_index = 1;
%     % Remove all the results for the faulty tests that were deleted
%     while (result_index <= size(results,2) && test_index <= size(removed_tests,2))
%        % Check if the result number matches the removed test number
%        if(results(1,result_index) == removed_tests(test_index))
%           % If so, delete the data and move to the next removed test
%           results(:,result_index) = []; 
%           test_index = test_index + 1;
%        else
%           % Otherwise, move to the next result
%           result_index = result_index + 1;
%        end
%     end

    % Removed all the faulty tests that were deleted
    for i=1:size(removed_tests,2)
       removed_test_index = find(results(1,:) == removed_tests(i),1);
       if (~isempty(removed_test_index))
          results(:,removed_test_index) = []; 
       end
    end
end       
    
results = sortrows(results')';
findOutliers(results);

results_file = strcat(test_path,'results.csv');
csvwrite(results_file,results)

loadResults(results_file);


