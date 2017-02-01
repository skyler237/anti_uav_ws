function [] = findlines()

addpath('~/')
addpath(genpath([pwd,'/..']));
data = processAllTopics('~/OA_collect.bag');
start = 400;
bun_radius = 1.2;
hot_dog_radius = 0.6;
max_velocity = 1.5;

for j = 1:length(data.scan.angle_min)-start;
    % pull in data from bag file
    index = j+1;
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
    
    % look for gaps in the data
    differences = diff(inlierIdx);
    starter = 1;
    finish = 1;
    while finish < length(inlierIdx)
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
            differences = diff((data(inlierIdx,1).^2 + data(inlierIdx,2).^2).^0.5);
        end
        k = k+1;
    end
    
    
    
end
end
