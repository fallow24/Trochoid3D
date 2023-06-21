%% Simulation parameters 
T = 500; % seconds, simulation time
step = 0.01; % seconds, stepsize 
nsteps = T/step+1;
t=0:step:T;
rs = 0.145; % m, radius of sphere
d = [0.00; 0.04; 0.07]; % m, extrinsic parameters Center -> Sensor 
% Input signal (angular velocity) in spheres frame of reference
wx = zeros(1,nsteps); %sin(t./(2*pi)); %uncomment to make spherical trochoid
wy = ones(1,nsteps); %-cos(t/(2*pi));
wz = zeros(1, nsteps); %zeros(1,nsteps);
w = [wx; wy; wz];
% Gravity vector wrt. global frame. 
g = 9.81*[0;0;1]; % m/s2
% Normal vector of plane wrt. global frame
n = [0;0;1]; % rotation around this axis will not result in translation.
% Initial angle in global reference frame
a0 = [0;0;45]; % degrees

%% Internal variables
dr = zeros(3,nsteps); % Position of sensor wrt. center of sphere
v = zeros(3,nsteps); % Lin. velocity of center of sphere over ground
c = zeros(3,nsteps); % Position of the center wrt. global frame
pos = zeros(3,nsteps); % Position of the sensor wrt. global frame
alpha = cell(1,nsteps); % Orientation of the ball 
% Initial conditions
alpha{:,1} = eul2rotm([a0(1) a0(2) a0(3)],'XYZ');
dr(:,1) = alpha{:,1}*d;
pos(:,1) = dr(:,1) + c(:,1);
for k = 1:1:nsteps-1 
    delR = eul2rotm((alpha{1,k}*w(:,k))'*step, 'XYZ');
    % Rotate sensor in fixed sphere-center frame
    alpha{1,k+1} = delR*alpha{1,k};
    % Integration of gyro measurement
    dr(:,k+1) = alpha{1,k+1}*dr(:,1);
    % Linear velocity (Roll without slip)
    v(:,k) = rs*cross(alpha{1,k}*w(:,k), n/norm(n));
    c(:,k+1) = c(:,k) + v(:,k)*step;
    pos(:,k+1) = dr(:,k+1) + c(:,k+1);
end
%% Plot
figure(1)

subplot(3,1,1)
plot(t, wx)
title('Rotation speed x-axis')
subplot(3,1,2)
plot(t,wy)
title('Rotation speed y-axis')
subplot(3,1,3)
plot(t,wz)
title('Rotation speed z-axis')

figure(2)
plot3(pos(1,:), pos(2,:), pos(3,:));
hold on
plot3(c(1,:), c(2,:), c(3,:));
hold off
grid on
title('Sensor Trajectory')
xlim([-10 70])
ylim([-10 70])
xlabel('X')
ylabel('Y')
zlabel('Z')

figure(3)
plot3(dr(1,:), dr(2,:), dr(3,:))
xlabel('X')
ylabel('Y')
zlabel('Z')
