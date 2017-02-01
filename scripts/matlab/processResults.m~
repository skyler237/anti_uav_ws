path(path,'skyler')

test_path = 'skyler/recent_tests/analysis/autotest/test16/';

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
   
   data = processAllTopics(strcat(test_path,bagfile));
   data = data.results;
   
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
results_file = strcat(test_path,'results.csv');
csvwrite(results_file,results)

format compact
findOutliers(results);


