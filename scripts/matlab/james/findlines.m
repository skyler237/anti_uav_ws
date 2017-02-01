function [] = findlines()

addpath('~/')
addpath(genpath([pwd,'/..']));
data = processAllTopics('~/OA_collect.bag');
start = 723;
bun_radius = 1.2;
hot_dog_radius = 0.6;
max_velocity = 1.5;

for j = 1:length(data.scan.angle_min)-start;
    index = j+start
    angle_min = data.scan.angle_min(index);
    increment = data.scan.angle_increment(index);
    angle_max = data.scan.angle_max(index);
    theta = (angle_min:increment:angle_max)';
    rho = data.scan.ranges(:,index);
    
    % remove unwanted data
    i = 1;
    while i<=length(rho)
        if not(isfinite(rho(i))) ||  rho(i) > bun_radius
            rho(i) = [];
            theta(i) = [];
        else
            i = i +1;
        end
    end
    
    if(length(rho) > 2)
        % convert to x-y
        x = rho.*cos(theta);
        y = rho.*sin(theta);
        
        incoming = [x,y];
        
        % Plot Incoming Data
        handle = figure(2)
        clf
        hold on
        plot(incoming(:,1),incoming(:,2), '.b')
        axis([-2,2,-2,2])
        axis('equal')
        hold off
        
        % run multiple ransac filter
        [m,b,inlierIdx] = multiple_ransac(incoming,3,5,.05,.2,.3);
        
        % deal with extra obstacles (not lines)
        inliers = incoming([inlierIdx(1,find(inlierIdx(1,:))),inlierIdx(2,find(inlierIdx(2,:))),inlierIdx(3,find(inlierIdx(3,:)))],:);
        outliers = incoming;
        outliers([inlierIdx(1,find(inlierIdx(1,:))),inlierIdx(2,find(inlierIdx(2,:))),inlierIdx(3,find(inlierIdx(3,:)))],:) = [];
        outlier_index = 1:1:length(incoming);
        outlier_index([inlierIdx(1,find(inlierIdx(1,:))),inlierIdx(2,find(inlierIdx(2,:))),inlierIdx(3,find(inlierIdx(3,:)))]) = [];
        
        % find normals
        normals = zeros(length(m)+2,2);
        % first, find normals to discovered lines
        for k = 1:length(m)
            if(length(find(inlierIdx(k,:)))>1)
                [closest_point, temp] = min(rho(inlierIdx(k,find(inlierIdx(k,:)))));
                closest_index = inlierIdx(k,temp);
                closest_angle = theta(closest_index);
                magnitude = max_velocity*(bun_radius-closest_point)/(bun_radius-hot_dog_radius)^1;
                start_point = [x(closest_index),y(closest_index)];
                final_point = [start_point(1) + magnitude*cos(closest_angle+pi()),...
                               start_point(2) + magnitude*sin(closest_angle+pi())];
                xp = [start_point(1);final_point(1)];
                yp = [start_point(2);final_point(2)];
                normals(k,:) = final_point - start_point;
%                 hold on
%                 plot(xp,yp)
%                 hold off
            end
        end
        
        % deal with outliers
        if length(outliers) > 3
%             hold on
            plot(outliers(:,1),outliers(:,2),'.r')
            [closest_point, temp] = min(rho(outlier_index));
            closest_index = outlier_index(temp);
            closest_angle = theta(closest_index);
            magnitude = max_velocity*(bun_radius-closest_point)/(bun_radius-hot_dog_radius)^1;
            start_point = [x(closest_index),y(closest_index)];
            final_point = [start_point(1) + magnitude*cos(closest_angle+pi()),...
                           start_point(2) + magnitude*sin(closest_angle+pi())];
            xp = [start_point(1);final_point(1)];
            yp = [start_point(2);final_point(2)];
%             plot(xp,yp)
%             hold off
            normals(k,:) = final_point - start_point;
        end
        
        % find largest command from any one normal
        largest = max(sqrt(normals(:,1).^2 + normals(:,2).^2));
        
        % Add Vectors
        resulting_command = [sum(normals(:,1)),sum(normals(:,2))];
        if norm(resulting_command) > largest
            resulting_command = largest * resulting_command/norm(resulting_command);
        end
        
        hold on
        plot([0,resulting_command(1)],[0,resulting_command(2)])
        hold off
        pause(.01);
        
            
    end
    
end

end


function [bestm, bestb, inlier_index] = multiple_ransac(data,num_lines,iter,threshDist,threshToss,inlierRatio)


model_points = 2; % two points for line model

number = size(data,1);
bestInNum = zeros(num_lines,1);
bestm = zeros(num_lines,1);
bestb = zeros(num_lines,1);
mask = ones(length(data),1);
inliers = [];
inlier_index = zeros(num_lines,length(data));
sample_history = zeros(num_lines,4);

for j = 1:num_lines
    i = 1;
    while i<iter && sum(mask) > 5
        % Choose a new sample of 2 points
        idx =  datasample(find(data(:,1) & mask),2); 
        sample = data(idx,:);
        % Determine the fit with the other data points
        kLine = sample(2,:)-sample(1,:);
        kLineNorm = kLine/norm(kLine);
        normVector = [-kLineNorm(2),kLineNorm(1)];
        distance = normVector*([mask,mask].*(data - repmat(sample(1,:),number,1)))';
        inlierIdx = find(and(abs(distance) <= threshDist,mask'));
        similarIdx = find(and(abs(distance) <= threshToss,mask'));
        
        
        % filter inliers to approve good models.
        % look for large gaps in the data
        differences = diff(inlierIdx);
        k = 1;
        while k < length(inlierIdx)
            if differences(k) > 10
                if k < length(inlierIdx)/2
                    inlierIdx(1:k) = [];
                    % make sure I don't cut off one of the samples
                    if idx(1) < k || idx(2) < k
                        inlierIdx = [];
                    end
                else
                    inlierIdx(k:end) = [];
                    if idx(1) > k || idx(2) > k
                        inlierIdx = [];
                    end
                end
                differences = diff(inlierIdx);
            end
            k = k+1;
        end
        
        inlierNum = length(inlierIdx);
        if inlierNum >= max(5,round(inlierRatio*length(find(mask)))) && inlierNum>bestInNum(j)
            inliers = similarIdx;
            sample_history(j,:) = [sample(1,:),sample(2,:)];
            bestInNum(j) = inlierNum;
            bestm(j) = (sample(2,2)-sample(1,2))/(sample(2,1)-sample(1,1));
            bestb(j) = sample(1,2)-bestm(j)*(sample(1,1));
            inlier_index(j,:) = zeros(1,length(inlier_index(1,:)));
            inlier_index(j,1:length(inliers)) = inliers';
        else if i == iter && length(inliers) < 0
                % no good lines found
                inliers = [];
            end
        end
        i = i+1;
    end
    % plot found l
    if(not(isempty(inliers)))
        lx = data(inliers,1);
        ly = bestm(j)*lx + bestb(j);
        hold on
        plot(lx,ly);
        mask(inliers) = 0;
        pause(0.001)
        inlierRatio = inlierRatio+inlierRatio;
        inliers = [];
        
    end
end

hold off
end