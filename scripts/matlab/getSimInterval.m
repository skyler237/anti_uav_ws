function [ sim_interval ] = getSimInterval( data, start, stop )
%getSimInterval - trims sim data to get a specified time interval 
%   A = 4xM array, where row 4 contains timestamp data
%   start, stop = starting and stopping time in seconds

index = 1;
time_row = size(data,1); % the time data is always on the last row
while index <= size(data,2)
   if  data(time_row, index) < start % If it's too early
      data(:, index) = []; % delete the column
   elseif data(time_row, index) > stop % If it's too late
      data(:, index) = []; % delete the column
   else
       index = index + 1; % otherwise, move on
   end
end

sim_interval = data;

end

