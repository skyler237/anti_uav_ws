path(path,'skyler')

test_path = 'skyler/recent_tests/analysis/autotest/test5_12-14-16/';

FileList = dir(strcat(test_path,'*.bag'));
N = size(FileList,1);
net_width = 6;

times = [];
points = [];
positions = [];
distances = [];
successes = [];


figure(1), clf
hold on
axis equal
rectangle('Position', [-net_width/2 -net_width/2 net_width, net_width])

for k=1:N
   bagfile = FileList(k).name;
   fprintf('Processing file: %s\n\r', bagfile);
   [isSuccess,int_point, int_position, time] = findIntercept(strcat(test_path,bagfile));
   
   if(isSuccess)
       plot(int_point(1), int_point(2),'g+')
   else
       plot(int_point(1), int_point(2),'r+')
   end
   
   successes = [successes,isSuccess];
   points = [points, int_point];
   positions = [positions, int_position];
   distances = [distances, norm(int_position)];
   times = [times,time];
   
end

fprintf('============ RESULTS =============\n');
fprintf('Out of %d simulations, %d were successful\n', N, sum(successes));
fprintf('Success rate = %f\n\n', sum(successes)/N);

fprintf('Average point = (%f, %f)\n', mean(points(1,:)), mean(points(2,:)));
fprintf('Average intercept radius = %f\n', mean(distances));
fprintf('Average intercept time = %f\n\n', mean(times));

results = [successes;points;positions;distances;times];
results_file = strcat(test_path,'results.csv');
csvwrite(results_file,results)


