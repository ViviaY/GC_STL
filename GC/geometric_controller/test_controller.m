%%
close all;
addpath('aux_functions');
addpath('test_functions');

%% load data
% traj_data = readtable("C:\Users\Beren\Downloads\STL_yt\yt\stlcg-1_3d_with_notB1.csv");
% traj_data = readtable("C:\Users\Beren\Desktop\STL\STL_rho_yt\STL_rho_yt\0.1-stlcg-3d.csv");
traj_data = readtable("C:\Users\Beren\Downloads\STL_yt\yt\Ztunnel.csv");
% traj_data = readtable("C:\Users\Beren\Downloads\STL_yt\yt\Ltunnel.csv");
% traj_data = readtable("C:\Users\Beren\Desktop\STL\STL_rho_yt\STL_rho_yt\stlcg-1_3d_seg9_yt.csv");
% traj_data = readtable("C:\Users\Beren\Downloads\STL_yt\yt\zigzag.csv");
traj_data = table2array(traj_data);
traj_data(:,1) = [];
% rho = traj_data(1,1);
traj_data(:,1) = [];
% plot3(traj_data(:,2),traj_data(:,3),traj_data(:,4));
traj_t = traj_data(:,1);
traj_x = traj_data(:,2);
traj_y = traj_data(:,3);
traj_z = traj_data(:,4);
traj_vx = traj_data(:,5);
traj_vy = traj_data(:,6);
traj_vz = traj_data(:,7);
traj_ax = traj_data(:,8);
traj_ay = traj_data(:,9);
traj_az = traj_data(:,10);

%% spline smoothing
% tt = traj_t(1):0.01:traj_t(end);
% xx = spline(traj_t,traj_x,tt);
% yy = spline(traj_t,traj_y,tt);
% zz = spline(traj_t,traj_z,tt);
% vxx = spline(traj_t,traj_vx,tt);
% vyy = spline(traj_t,traj_vy,tt);
% vzz = spline(traj_t,traj_vz,tt);
% axx = spline(traj_t,traj_ax,tt);
% ayy = spline(traj_t,traj_ay,tt);
% azz = spline(traj_t,traj_az,tt);

tt = traj_t(1):0.01:traj_t(end);
% xx = pchip(traj_t,traj_x,tt);
% yy = pchip(traj_t,traj_y,tt);
% zz = pchip(traj_t,traj_z,tt);
% vxx = pchip(traj_t,traj_vx,tt);
% vyy = pchip(traj_t,traj_vy,tt);
% vzz = pchip(traj_t,traj_vz,tt);
% axx = pchip(traj_t,traj_ax,tt);
% ayy = pchip(traj_t,traj_ay,tt);
% azz = pchip(traj_t,traj_az,tt);

% des_traj = [tt' xx' yy' zz' vxx' vyy' vzz' axx' ayy' azz'];
% des_traj = [tt' xx' yy' zz'];


%% Simulation parameters
t = tt;
N = length(t);

% Quadrotor
J1 = 0.02;
J2 = 0.02;
J3 = 0.04;
param.J = diag([J1, J2, J3]);

param.m = 2;

param.d = 0.169;
param.ctf = 0.0135;

% Fixed disturbance
param.x_delta = [0,0,0]';
param.R_delta = [0,0,0]';

% Other parameters
param.g = 9.8;

%% Controller gains
k.x = 10;
k.v = 8;
k.i = 10;
param.c1 = 1.5;
param.sigma = 10;

% Attitude
k.R = 1.5;
k.W = 0.35;
k.I = 10;
param.c2 = 2;

% Yaw
k.y = 0.8;
k.wy = 0.15;
k.yI = 2;
param.c3 = 2;

%% Initial conditions
% x0 = [0, 0, 0]';
rho = 0.2;
delta1 = (rho/sqrt(3))*rand(1);
delta2 = -(rho/sqrt(3))*rand(1);
delta3 = (rho/sqrt(3))*rand(1);
x0 = [traj_data(1,2), traj_data(1,3)-0.1, traj_data(1,4)+0.1]';
v0 = [0, 0, 0]';
R0 = expm(pi * hat([0, 0, 1]'));
W0 = [0, 0, 0]';
X0 = [x0; v0; W0; reshape(R0,9,1); zeros(6,1)];

%% Numerical integration
tStart = tic;
[t, X] = ode45(@(t, XR) eom(t, XR, k, param, traj_data), t, X0, ...
    odeset('RelTol', 1e-6, 'AbsTol', 1e-6));
tEnd = toc(tStart);

%% Post processing

% Create empty arrays to save data
[e, d, R, f, M] = generate_output_arrays(N);

% Unpack the outputs of ode45 function
x = X(:, 1:3)';
v = X(:, 4:6)';
W = X(:, 7:9)';
ei = X(:, 19:21)';
eI = X(:, 22:24)';

for i = 1:N
    R(:,:,i) = reshape(X(i,10:18), 3, 3);
    
    des = command_interpolate(t(i), traj_data);

    [f(i), M(:,i), ~, ~, err, calc] = position_control(X(i,:)', des, ...
        k, param);
    
    % Unpack errors
    e.x(:,i) = err.x;
    e.v(:,i) = err.v;
    e.R(:,i) = err.R;
    e.W(:,i) = err.W;
    e.y(i) = err.y;
    e.Wy(i) = err.Wy;
    
    % Unpack desired values
    d.x(:,i) = des.x;
    d.v(:,i) = des.v;
    d.b1(:,i) = des.b1;
    d.R(:,:,i) = calc.R;
end

% Plot data
% linetype = 'k';
% linewidth = 1;
% xlabel_ = 'time (s)';
% 
% figure;
% plot_3x1(t, e.R, '', xlabel_, 'e_R', linetype, linewidth)
% set(gca, 'FontName', 'Times New Roman');
% 
% figure;
% plot_3x1(t, e.x, '', xlabel_, 'e_x', linetype, linewidth)
% set(gca, 'FontName', 'Times New Roman');
% 
% figure;
% plot_3x1(t, e.v, '', xlabel_, 'e_v', linetype, linewidth)
% set(gca, 'FontName', 'Times New Roman');
% 
% figure;
% plot_3x1(t, eI .* [k.I, k.I, k.yI]', '', xlabel_, 'e', linetype, linewidth)
% plot_3x1(t, param.R_delta .* ones(3, N), ...
%     '', xlabel_, 'e_I', 'r', linewidth)
% set(gca, 'FontName', 'Times New Roman');
% 
% figure;
% plot_3x1(t, ei * k.i, '', xlabel_, 'e_i', linetype, linewidth)
% plot_3x1(t, param.x_delta .* ones(3, N), ...
%     '', xlabel_, 'e_i', 'r', linewidth)
% set(gca, 'FontName', 'Times New Roman');
% 
% figure;
% plot_3x1(t, x, '', xlabel_, 'x', linetype, linewidth)
% plot_3x1(t, d.x, '', xlabel_, 'x', 'r', linewidth)
% set(gca, 'FontName', 'Times New Roman');
% 
% figure;
% plot3(x(1,:), x(2,:), x(3,:), 'k');
% hold on;
% plot3(d.x(1,:), d.x(2,:), d.x(3,:), 'r');
% set(gca, 'YDir', 'reverse', 'ZDir', 'reverse');
% axis equal;
% xlabel('$x_1$', 'interpreter', 'latex');
% ylabel('$x_2$', 'interpreter', 'latex');
% zlabel('$x_3$', 'interpreter', 'latex');
% set(gca, 'Box', 'on');
% grid on;
% set(gca, 'FontName', 'Times New Roman');

plot3(x(1,:), x(2,:), x(3,:));
hold on;
plot3(traj_data(:,2),traj_data(:,3),traj_data(:,4));