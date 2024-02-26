%% Simulation parameters 
rs = 0.145; % m, radius of sphere
d = [0.00; 0.04; -0.07]; % m, extrinsic parameters Center -> Sensor 
% Normal vector of plane wrt. global frame
n = [0;0;-1]; % rotation around this axis will not result in translation.

% Input signal (angular velocity) in spheres frame of reference
bag = rosbag('/home/fabi/Documents/bagfiles/TimRolling/20_02_large01.bag');
topic = select(bag, 'Time', [bag.StartTime, bag.EndTime], 'Topic', '/orientation');
msgs = readMessages(topic, 'DataFormat', 'struct');
nsteps = length(msgs);
t = cellfun( @(m) double(m.Header.Stamp.Nsec), msgs)';
dt = zeros(1,nsteps); % time deltas
ts = zeros(1,nsteps); % time in seconds
countNumSec = 0; % count nanosecond overflows
for i = 1:nsteps-1
    dt(1,i) = t(1,i+1) - t(1,i);
    ts(1,i) = t(1,i)*1e-9 + countNumSec;
    if (dt(1,i) < 0) % nanosecond overflow possible
        dt(1,i) = dt(1,i) + 1e9; % if that happens, add one second
        countNumSec = countNumSec + 1;
    end
    dt(1,i) = 1e-9 * dt(1,i); % convert to seconds
end
% Timestamps
ts(1,nsteps) = t(1,nsteps)*1e-9 + countNumSec; %trailing zero
% Gyro vals
wx = cellfun( @(m) double(m.AngularVelocity.X), msgs)';
wy = cellfun( @(m) double(m.AngularVelocity.Y), msgs)';
wz = cellfun( @(m) double(m.AngularVelocity.Z), msgs)';
w = [wx; wy; wz]*180/pi; % given in degrees/s
% Pre-filtered orientation
qx = cellfun( @(m) double(m.Orientation.X), msgs)';
qy = cellfun( @(m) double(m.Orientation.Y), msgs)';
qz = cellfun( @(m) double(m.Orientation.Z), msgs)';
qw = cellfun( @(m) double(m.Orientation.W), msgs)';
% conversion from ros to matlab quaternions:
q = [-qw; qx; qy; qz];

%% Internal variables
dr = zeros(3,nsteps); % Position of sensor wrt. center of sphere
v = zeros(3,nsteps); % Lin. velocity of center of sphere over ground
c = zeros(3,nsteps); % Position of the center wrt. global frame
pos = zeros(3,nsteps); % Position of the sensor wrt. global frame
alpha = cell(1,nsteps); % Orientation of the ball 
% Initial conditions
alpha{1,1} = quat2rotm(q(:,1)');
dr(:,1) = alpha{1,1}\d;
pos(:,1) = dr(:,1) + c(:,1);
v(:,1) = cross(alpha{1,1}\(rs*w(:,1)*pi/180), n/norm(n));
%% Simulation
for k = 1:1:nsteps-1 
    % Rotate sensor in fixed sphere-center frame
    alpha{1,k+1} = quat2rotm(q(:,k+1)');
    % Integration of gyro measurement
    dr(:,k+1) = alpha{1,k+1}\d;
    % Linear velocity (Roll without slip)
    v(:,k+1) = cross(alpha{1,k+1}\(rs*w(:,k+1)*pi/180), n/norm(n));
    c(:,k+1) = c(:,k) + v(:,k)*dt(:,k);
    pos(:,k+1) = dr(:,k+1) + c(:,k+1);
end

%% Plot
figure(1)

subplot(3,1,1)
plot(ts, wx)
title('Rotation speed x-axis')
xlabel('t in s')
ylabel('Ang. vel. in rad/s')
subplot(3,1,2)
plot(ts,wy)
title('Rotation speed y-axis')
xlabel('t in s')
ylabel('Ang. vel. in rad/s')
subplot(3,1,3)
plot(ts,wz)
title('Rotation speed z-axis')
xlabel('t in s')
ylabel('Ang. vel. in rad/s')

figure(2)
plot3(pos(1,:), pos(2,:), pos(3,:));
hold on
plot3(c(1,:), c(2,:), c(3,:));
hold off
grid on
title('Sensor and Sphere Center Trajectory wrt global frame')
xlim([-3 3])
ylim([-1 5])
xlabel('X')
ylabel('Y')
zlabel('Z')

figure(3)
plot3(dr(1,:), dr(2,:), dr(3,:))
grid on
title('Sensor Trajectory wrt sphere center')
xlabel('X')
ylabel('Y')
zlabel('Z')

