addpath('~/')
data = processAllTopics('~/OA_collect.bag');

index = 930;
angle_min = data.scan.angle_min(index);
increment = data.scan.angle_increment(index);
angle_max = data.scan.angle_max(index);
theta = (angle_min:increment:angle_max)';
rho = data.scan.ranges(:,index);
i = 1;
while i<=length(rho)
    if not(isfinite(rho(i)))
        rho(i) = [];
        theta(i) = [];
    else
        i = i + 1;
    end
end


figure(1)
clf
hold on
polar(theta,rho)

[b,a] = butter(10,.2);
rho_filt = filtfilt(b,a,double(rho));

polar(theta,rho_filt)
hold off

