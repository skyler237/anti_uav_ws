% RANSACFITLINE - fits line to 3D array of points using RANSAC
%
% Usage  [L, inliers] = ransacfitline(XYZ, t, feedback)
%
% This function uses the RANSAC algorithm to robustly fit a line
% to a set of 3D data points.
%
% Arguments:
%          XYZ - 3xNpts array of xyz coordinates to fit line to.
%          t   - The distance threshold between data point and the line
%                used to decide whether a point is an inlier or not.
%          feedback - Optional flag 0 or 1 to turn on RANSAC feedback
%                     information.
%
% Returns:.
%           V - Line obtained by a simple fitting on the points that
%               are considered inliers.  The line goes through the
%               calculated mean of the inlier points, and is parallel to
%               the principal eigenvector.  The line is scaled by the
%               square root of the largest eigenvalue.
%               This line is a n*2 matrix.  The first column is the
%               beginning point, the second column is the end point of the
%               line.
%           L - The two points in the data set that were found to
%               define a line having the most number of inliers.
%               The two columns of L defining the two points.
%           inliers - The indices of the points that were considered
%                     inliers to the fitted line.
%
% See also:  RANSAC, FITPLANE, RANSACFITPLANE

function [V, L, inliers] = ransacfitline(XYZ, t, feedback)
    
    if nargin == 2
	feedback = 0;
    end
    
    [rows, npts] = size(XYZ);
    
    if rows ~=3
        error('data is not 3D');
    end
    
    if npts < 2
        error('too few points to fit line');
    end
    
    s = 2;  % Minimum No of points needed to fit a line.
        
    fittingfn = @defineline;
    distfn    = @lineptdist;
    degenfn   = @isdegenerate;

    [L, inliers] = ransac(XYZ, fittingfn, distfn, degenfn, s, t, feedback);
    
    % Find the line going through the mean, parallel to the major
    % eigenvector
    V = fitline3d(XYZ(:, inliers));
    
%------------------------------------------------------------------------
% Function to define a line given 2 data points as required by
% RANSAC.

function L = defineline(X);
    L = X;
    
%------------------------------------------------------------------------
% Function to calculate distances between a line and an array of points.
% The line is defined by a 3x2 matrix, L.  The two columns of L defining
% two points that are the endpoints of the line.
%
% A line can be defined with two points as:
%        lambda*p1 + (1-lambda)*p2
% Then, the distance between the line and another point (p3) is:
%        norm( lambda*p1 + (1-lambda)*p2 - p3 )
% where
%                  (p2-p1).(p2-p3)
%        lambda =  ---------------
%                  (p1-p2).(p1-p2)
%
% lambda can be found by taking the derivative of:
%      (lambda*p1 + (1-lambda)*p2 - p3)*(lambda*p1 + (1-lambda)*p2 - p3)
% with respect to lambda and setting it equal to zero

function [inliers, L] = lineptdist(L, X, t)

    p1 = L(:,1);
    p2 = L(:,2);
    
    npts = length(X);
    d = zeros(npts, 1);
    
    for i = 1:npts
        p3 = X(:,i);
      
        lambda = dot((p2 - p1), (p2-p3)) / dot( (p1-p2), (p1-p2) );
        
        d(i) = norm(lambda*p1 + (1-lambda)*p2 - p3);
    end
    
    inliers = find(abs(d) < t);
    
%------------------------------------------------------------------------
% Function to determine whether a set of 2 points are in a degenerate
% configuration for fitting a line as required by RANSAC.
% In this case two points are degenerate if they are the same point
% or if they are exceedingly close together.

function r = isdegenerate(X)
    %find the norm of the difference of the two points
    % this will be 0 iff the two points are the same (the norm of their
    % difference is zero)
    r = norm(X(:,1) - X(:,2)) < eps;



















 function [bestParameter1,bestParameter2] = ransac_demo(data,num,iter,threshDist,inlierRatio)
 % data: a 2xn dataset with #n data points
 % num: the minimum number of points. For line fitting problem, num=2
 % iter: the number of iterations
 % threshDist: the threshold of the distances between points and the fitting line
 % inlierRatio: the threshold of the number of inliers 
 
 %% Plot the data points
 figure;plot(data(1,:),data(2,:),'o');hold on;
 number = size(data,2); % Total number of points
 bestInNum = 0; % Best fitting line with largest number of inliers
 bestParameter1=0;bestParameter2=0; % parameters for best fitting line
 for i=1:iter
 %% Randomly select 2 points
     idx = randperm(number,num); sample = data(:,idx);   
 %% Compute the distances between all points with the fitting line 
     kLine = sample(:,2)-sample(:,1);
     kLineNorm = kLine/norm(kLine);
     normVector = [-kLineNorm(2),kLineNorm(1)];
     distance = normVector*(data - repmat(sample(:,1),1,number));
 %% Compute the inliers with distances smaller than the threshold
     inlierIdx = find(abs(distance)<=threshDist);
     inlierNum = length(inlierIdx);
 %% Update the number of inliers and fitting model if better model is found     
     if inlierNum>=round(inlierRatio*number) && inlierNum>bestInNum
         bestInNum = inlierNum;
         parameter1 = (sample(2,2)-sample(2,1))/(sample(1,2)-sample(1,1));
         parameter2 = sample(2,1)-parameter1*sample(1,1);
         bestParameter1=parameter1; bestParameter2=parameter2;
     end
 end
 
 %% Plot the best fitting line
 xAxis = -number/2:number/2; 
 yAxis = bestParameter1*xAxis + bestParameter2;
 plot(xAxis,yAxis,'r-','LineWidth',2);