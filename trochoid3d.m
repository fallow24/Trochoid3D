%% Simulation parameters 
T = 120; % seconds, simulation time
step = 0.01; % seconds, stepsize 
nsteps = T/step+1;
t=0:step:T;
rs = 0.145; % m, radius of sphere
rx = 0.04119993; % m, measured radius when rotating around x 
ry = 0.05758807; % m, measured radius when rotating around y 
rz = 0.01860155; % m, measured radius when rotating around z 
% Input signal (angular velocity) 
wx = zeros(1,nsteps); %sin(t./(2*pi)); uncomment to make spherical trochoid
wy = zeros(1,nsteps); %-cos(t/(2*pi));
wz = ones(1,nsteps); %zeros(1,nsteps);

%% Internal variables
x=zeros(1,nsteps);
y=zeros(1,nsteps);
z=zeros(1,nsteps);
yx=zeros(1,nsteps);
zx=zeros(1,nsteps);
xz=zeros(1,nsteps);
xy=zeros(1,nsteps);
zy=zeros(1,nsteps);
yz=zeros(1,nsteps);
alpha=zeros(3,nsteps);
for k = 2:1:nsteps
    % Get current phase in each rotation axis 
    alpha(1,k+1) = alpha(1,k) + wx(1,k)*step;
    alpha(2,k+1) = alpha(2,k) + wy(1,k)*step;
    alpha(3,k+1) = alpha(3,k) + wz(1,k)*step;
    % Rotation around x-axis:
    yx(1,k) = rs*alpha(1,k)-rx*sin(alpha(1,k));
    zx(1,k) = rs - rx*cos(alpha(1,k));
    % Rotation around y-axis:
    xy(1,k) = rs*alpha(2,k)-ry*sin(alpha(2,k));
    zy(1,k) = rs - ry*cos(alpha(2,k));
    % Rotation around z-axis:
    xz(1,k) = rs - rz*sin(alpha(3,k));
    yz(1,k) = rs - rz*cos(alpha(3,k));
    % Superposition
    x(1,k+1) = x(1,k) + xy(1,k) + xz(1,k) - xy(1,k-1) - xz(1,k-1);
    y(1,k+1) = y(1,k) + yx(1,k) + yz(1,k) - yx(1,k-1) - yz(1,k-1);
    z(1,k+1) = z(1,k) + zx(1,k) + zy(1,k) - zx(1,k-1) - zy(1,k-1);
end
x = x - rs;
y = y - rs;
z = z - rs;
x(1)=x(3);
x(2)=x(3);
y(1)=y(3);
y(2)=y(3);
z(1)=z(3);
z(2)=z(3);

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
plot3(x, y, z)
grid on
title('Sensor Trajectory')
xlabel('X')
ylabel('Y')
zlabel('Z')
