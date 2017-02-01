% rosbag record /whirlybird /command -O bagname.bag
bagfile = 'skyler/bagname.bag';

roll_meas = [data.whirlybird.roll; data.whirlybird.time];
pitch_meas = [data.whirlybird.roll; data.whirlybird.time];
yaw_meas = [data.whirlybird.roll; data.whirlybird.time];

left_motor = [data.command.left_motor; data.command.time];
right_motor = [data.command.right_motor; data.command.time];

