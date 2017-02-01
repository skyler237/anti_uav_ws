path(path,'skyler');
% test_path = 'skyler/recent_tests/analysis/autotest/test7_1-12-17/';
test_path = 'skyler/recent_tests/analysis/autotest/test12b/';

results_file = strcat(test_path,'results.csv');
results = csvread(results_file);

N = size(results,2);
% successes   =   results(1,:);
% points      =   results(2:3,:);
% positions   =   results(5:7,:);
% distances   =   results(8,:);
% times       =   results(9,:);

% % Use this for tests 13 and later
test_numbers =  results(1,:);
successes   =   results(2,:);
points      =   results(3:4,:);
positions   =   results(6:8,:);
distances   =   results(9,:);
times       =   results(10,:);


net_width = 6;

figure(1), clf
hold on
axis equal
rectangle('Position', [-net_width/2 -net_width/2 net_width, net_width])

% Remove all NaN entries
i=1;
while(i <= size(points,2))
   if (isnan(points(1,i)) || isnan(points(2,i)))
       
       results(:,i) = [];
       test_numbers(:,i) = [];
       successes(:,i) = [];
       points(:,i) = [];
       positions(:,i) = [];
       distances(:,i) = [];
       times(:,i) = [];
       
   else
       i = i + 1;
   end   
end

for k=1:size(successes,2)
   
   isSuccess = successes(k);
   int_point = points(:,k);
   
   if(isSuccess)
       plot(int_point(1), int_point(2),'g+')
   else
       plot(int_point(1), int_point(2),'r+')
   end
      
end



N = size(results,2);

fprintf('============ RESULTS =============\n');
fprintf('Out of %d simulations, %d were successful\n', N, sum(successes));
fprintf('Success rate = %f\n\n', sum(successes)/N);

fprintf('Average point = (%f, %f)\n', mean(points(1,:)), mean(points(2,:)));
fprintf('Average intercept time = %f\n', mean(times));
fprintf('Average intercept radius = %f\n\n', mean(distances));

format compact
findOutliers(results);