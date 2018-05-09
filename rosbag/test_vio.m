%vio comparison
%% Load rosbag file
bag_obj = rosbag('rosbag_1.bag');
%Extract all
odom_all = readMessages(select(bag_obj, 'Topic', '/odom'));
imu_all = readMessages(select(bag_obj, 'Topic',  '/imu'));
vio_all = readMessages(select(bag_obj, 'Topic',  '/optical_flow'));
vio_ekf_all = readMessages(select(bag_obj, 'Topic',  '/odometry/visual'));
imu_ekf_all = readMessages(select(bag_obj, 'Topic',  '/odometry/imu'));
%% Generate matlab vectors
odom_count = size(odom_all, 1);
imu_count = size(imu_all, 1);
vio_count = size(vio_all, 1);
vio_ekf_count = size(vio_ekf_all, 1);
imu_ekf_count = size(imu_ekf_all, 1);

odom = zeros(odom_count, 2);
odom_time = zeros(odom_count, 1);

imu_quaternion = zeros(imu_count, 4);
imu_pry = zeros(imu_count, 3);
imu_acc = zeros(imu_count, 3);
imu_time = zeros(imu_count, 1);

vio_vx_yaw = zeros(vio_count, 2);
vio_time = zeros(vio_count, 1);

vio_ekf_vx_yaw = zeros(vio_ekf_count, 2);
vio_ekf_time = zeros(vio_ekf_count, 1);

imu_ekf_vx_yaw = zeros(imu_ekf_count, 2);
imu_ekf_time = zeros(imu_ekf_count, 1);

odom_time_init = odom_all{1}.Header.Stamp.Sec;
imu_time_init = imu_all{1}.Header.Stamp.Sec;
vio_time_init = vio_all{1}.Header.Stamp.Sec;
vio_ekf_time_init = vio_ekf_all{1}.Header.Stamp.Sec;
imu_ekf_time_init = imu_ekf_all{1}.Header.Stamp.Sec;

for i = 1:odom_count
    odom(i,:) = [odom_all{i}.Twist.Twist.Linear.X, odom_all{i}.Twist.Twist.Angular.Z];
    if abs(odom(i,2)) > 1
        odom(i,:) = odom(i-1,:);
    end
    odom_time(i,1) = odom_all{i}.Header.Stamp.Sec - odom_time_init + odom_all{i}.Header.Stamp.Nsec * 1e-9;
end
encoder_yaw = cumtrapz(odom_time, odom(:,2));

for i = 1:imu_count
    imu_quaternion(i,:) = [imu_all{i}.Orientation.X, imu_all{i}.Orientation.Y, imu_all{i}.Orientation.Z, imu_all{i}.Orientation.W];
    imu_pry(i,:) = quat2eul(imu_quaternion(i,:));
    imu_acc(i,:) = [imu_all{i}.LinearAcceleration.X, imu_all{i}.LinearAcceleration.Y, imu_all{i}.LinearAcceleration.Z];
    imu_time(i,1) = imu_all{i}.Header.Stamp.Sec - imu_time_init + imu_all{i}.Header.Stamp.Nsec * 1e-9;
end
imu_vx = cumtrapz(imu_time, imu_acc(:,1));

for i = 1:vio_count
    vio_vx_yaw(i,:) = [vio_all{i}.Twist.Twist.Linear.X, vio_all{i}.Twist.Twist.Angular.Z*2];
    vio_time(i,1) = vio_all{i}.Header.Stamp.Sec - vio_time_init + vio_all{i}.Header.Stamp.Nsec * 1e-9;
end
vio_yaw = cumtrapz(vio_time, vio_vx_yaw(:,2));

for i = 1:vio_ekf_count
    vio_ekf_vx_yaw(i,:) = [vio_ekf_all{i}.Twist.Twist.Linear.X, vio_ekf_all{i}.Twist.Twist.Angular.Z*2];
    vio_ekf_time(i,1) = vio_ekf_all{i}.Header.Stamp.Sec - vio_time_init + vio_ekf_all{i}.Header.Stamp.Nsec * 1e-9;
end
vio_ekf_yaw = cumtrapz(vio_ekf_time, vio_ekf_vx_yaw(:,2));

for i = 1:imu_ekf_count
    imu_ekf_vx_yaw(i,:) = [imu_ekf_all{i}.Twist.Twist.Linear.X, imu_ekf_all{i}.Twist.Twist.Angular.Z*2];
    imu_ekf_time(i,1) = imu_ekf_all{i}.Header.Stamp.Sec - vio_time_init + imu_ekf_all{i}.Header.Stamp.Nsec * 1e-9;
end
imu_ekf_yaw = cumtrapz(imu_ekf_time, imu_ekf_vx_yaw(:,2));
%% plot yaw angle
figure;
plot(imu_time, unwrap(imu_pry(:,3)));
title("Yaw");
xlabel("Time (s)");
ylabel("Yaw (Rad)");
hold on;
plot(odom_time, unwrap(encoder_yaw));
plot(vio_time, unwrap(vio_yaw));
legend("IMU", "Encoder", "VIO")
figure;
plot(imu_time, unwrap(imu_pry(:,3)));
title("Yaw");
xlabel("Time (s)");
ylabel("Yaw (Rad)");
hold on;
plot(odom_time, unwrap(encoder_yaw));
plot(imu_ekf_time, unwrap(imu_ekf_yaw));
plot(vio_ekf_time, unwrap(vio_ekf_yaw));
legend("IMU", "Encoder", "IMU EKF", "IMU VIO EKF")
%% plot x velocity
figure;
plot(imu_time, imu_vx);
title("X Velocity");
xlabel("Time (s)");
ylabel("Speed (m/s)");
hold on;
plot(odom_time, odom(:,1));
plot(vio_time, vio_vx_yaw(:,1));
legend("IMU", "Encoder", "VIO")
figure;
plot(imu_time, imu_vx);
title("X Velocity");
xlabel("Time (s)");
ylabel("Speed (m/s)");
hold on;
plot(odom_time, odom(:,1));
plot(imu_ekf_time, imu_ekf_vx_yaw(:,1));
plot(vio_ekf_time, vio_ekf_vx_yaw(:,1));
legend("IMU", "Encoder", "IMU EKF", "IMU VIO EKF")
% %Integrate using Encoder
% theta = encoder_yaw;
% %integrate to get s
% v_x = odom(:,1) .* cos(theta);
% v_y = odom(:,1) .* sin(theta);
% s_x = cumtrapz(odom_time, v_x);
% s_y = cumtrapz(odom_time, v_y);
% figure;
% plot(s_x, s_y);
% %Integrate using IMU
% theta2 = imu_pry(:,3);
% %integrate to get vx
% v_x2 = cumtrapz(imu_time, imu_acc(:,2));
% %integrate to get the path
% s = cumtrapz(imu_time, v_x2);
% s_delta = diff(s);
% s_delta = cat(1, 0, s_delta);
% s_x_delta = s_delta .* cos(theta2);
% s_y_delta = s_delta .* sin(theta2);
% s_x2 = cumsum(s_x_delta);
% s_y2 = cumsum(s_y_delta);
% figure;
% plot(s_x2, s_y2);