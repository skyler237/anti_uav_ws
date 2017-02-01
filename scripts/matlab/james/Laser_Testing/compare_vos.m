%% Load

addpath('~/.ros/')
addpath(genpath('~/scripts/matlab'))
data = processAllTopics('~/.ros/cmu2.bag');

%% Process CMU

figure(1)
clf
subplot(311)
hold on
plot(data.cmu_state.time,data.cmu_state.transform.translation(1,:),'.r');
plot(data.truth_state.time,data.truth_state.transform.translation(1,:),'.b');
legend('rgbd','cortex')
hold off

subplot(312)
hold on
plot(data.cmu_state.time,data.cmu_state.transform.translation(2,:),'.r');
plot(data.truth_state.time,data.truth_state.transform.translation(2,:),'.b');
legend('rgbd','cortex')
hold off

subplot(313)
hold on
plot(data.vo_laser.time,data.vo_laser.transform.translation(2,:),'.r');
plot(data.vo_rgbd.time,data.vo_rgbd.transform.translation(1,:),'.b');
plot(data.vo_cortex.time,data.vo_cortex.transform.translation(1,:),'.g');
legend('rgbd','cortex')
hold off

%% Process Laser

figure(2)
clf
subplot(311)
hold on
plot(data.laser_state.time,data.laser_state.transform.translation(1,:),'.r');
plot(data.truth_state.time,data.truth_state.transform.translation(1,:),'.b');
legend('laser','cortex')
hold off

subplot(312)
hold on
plot(data.laser_state.time,data.laser_state.transform.translation(2,:),'.r');
plot(data.truth_state.time,data.truth_state.transform.translation(2,:),'.b');
legend('laser','cortex')
hold off

subplot(313)
hold on
plot(data.vo_laser.time,data.vo_laser.transform.translation(2,:),'.r');
plot(data.vo_rgbd.time,data.vo_rgbd.transform.translation(1,:),'.b');
plot(data.vo_cortex.time,data.vo_cortex.transform.translation(1,:),'.g');
legend('rgbd','cortex')
hold off