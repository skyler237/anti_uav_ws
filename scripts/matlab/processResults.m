path(path,'skyler')

test_path = 'skyler/recent_tests/analysis/autotest/test11/';

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
   fprintf('Processing file: %s\n\r', strcat(test_path,bagfile));
   
   try
    data = processAllTopics(strcat(test_path,bagfile));
    data = data.results;
   catch
    continue
   end
   
   strings = split(bagfile, {'results','.bag'});
   test_number = str2double(strings(2));
   isSuccess = data.isIntercept;
   int_point = [data.intercept_point];
   int_position = [data.intercept_pose];
   distance = data.intercept_radius;
   time = data.intercept_time;
   
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

fprintf('============ RESULTS =============\n');
fprintf('Out of %d simulations, %d were successful\n', N, sum(successes));
fprintf('Success rate = %f\n\n', sum(successes)/N);

fprintf('Average point = (%f, %f)\n', mean(points(1,:)), mean(points(2,:)));
fprintf('Average intercept time = %f\n', mean(times));
fprintf('Average intercept radius = %f\n\n', mean(distances));

results = [test_numbers;successes;points;positions;distances;times];

format compact
[outliers, success_outliers, failed_inliers, ...
    possible_failed_mistakes, possible_success_mistakes] = findOutliers(results);

removed_tests = [removeFaultyTests(test_path, possible_failed_mistakes, possible_success_mistakes),...
                removeFaultyTests(test_path, outliers, success_outliers)];
            
result_index = 1;
test_index = 1;
% Remove all the results for the faulty tests that were deleted
while (result_index <= size(results,2) && test_index <= size(removed_tests,2))
   % Check if the result number matches the removed test number
   if(results(1,result_index) == removed_tests(test_index))
      % If so, delete the data and move to the next removed test
      results(:,result_index) = []; 
      test_index = test_index + 1;
   else
      % Otherwise, move to the next result
      result_index = result_index + 1;
   end
end

results = sortrows(results')';
results_file = strcat(test_path,'results.csv');
csvwrite(results_file,results)


