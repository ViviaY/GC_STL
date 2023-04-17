% this can run

% This source code is written to implement flight simulations for one quadrotor
% Author: wjxjmj
% Email: wjxjmj@126.com
% Open Source License: GPL

% shall we go!
clear all
clc

traj_data = readtable("C:\Users\Beren\Downloads\STL_yt\yt\stlcg-1_3d_with_notB1.csv");
% traj_data = readtable("C:\Users\Beren\Desktop\STL\STL_rho_yt\STL_rho_yt\0.1-stlcg-3d.csv");
% traj_data = readtable("C:\Users\Beren\Downloads\STL_yt\yt\Ztunnel.csv");
% traj_data = readtable("C:\Users\Beren\Downloads\STL_yt\yt\Ltunnel.csv");
% traj_data = readtable("C:\Users\Beren\Desktop\STL\STL_rho_yt\STL_rho_yt\stlcg-1_3d_seg9_yt.csv");
% traj_data = readtable("C:\Users\Beren\Downloads\STL_yt\yt\zigzag.csv");
traj_data = table2array(traj_data);
traj_data(:,1) = [];
traj_data(:,1) = [];
% plot3(traj_data(:,2),traj_data(:,3),traj_data(:,4));

% simulation paraments set up
dt=0.01;
stime=19.5;
loop=stime/dt;

real_traj = zeros(3,loop);
ref_traj = zeros(3,loop);

% flocking paraments set up
d=3;
n=1;

% init state
s=zeros(12,n);
s(1:3,:)=traj_data(1,2:4)';
s(4:6,:)=unifrnd(-0,0,[3,n]);
s(7,:)=unifrnd(-0.0*pi,0.0*pi,[1,n]);
s(8,:)=unifrnd(-0.0*pi,0.0*pi,[1,n]);
s(9,:)=unifrnd(-0.0*pi,0.0*pi,[1,n]);
x=s(1);y=s(2);z=s(3);
vx=s(4);vy=s(5);vz=s(6);
phi=s(7);theta=s(8);psi=s(9);
vphi=s(10);vtheta=s(11);vpsi=s(12);

% public virtual leadr init
xl=traj_data(1,2:4)';
vl=[0;0;0];

%parameters for quadrotor
para.g=9.8;
para.m=2;
para.Iy=0.02;
para.Ix=0.02;
para.Iz=0.04;
% para.b=10^-4;
para.b=0.169;
para.l=0.5;
% para.d=10^-6;
para.d=0.0135;
para.Jr=0.01;
para.k1=0.02;
para.k2=0.02;
para.k3=0.02;
para.k4=0.1;
para.k5=0.1;
para.k6=0.1;
para.omegaMax=330;

% history capture
xyHis=zeros(d,n+1,loop+1);
xyHis(:,:,1)=[xl s(1:3)];

%simulation start
% hwait=waitbar(0,'½øÐÐÖÐ>>>>>>>>>>');

sp=1;
omegaHis=zeros(4,loop);

tStart = tic;
for t=1:loop
    
    %leader information generator
    % if t/loop<0.1
    %     al=([0;0;sp]-vl);
    % elseif t/loop<0.2
    %     al=([sp;0;0]-vl);
    % elseif t/loop<0.4
    %     al=([0;sp;0]-vl);
    % elseif t/loop<0.6
    %     al=([-sp;0;0]-vl);
    % elseif t/loop<0.8
    %     al=([0;-sp;0]-vl);
    % elseif t/loop<0.9
    %     al=([sp;0;sp]-vl);
    % else
    %     al=([sp;0;0]-vl);
    % end
    index = 0;
    for i = 1:size(traj_data,1)
        if t * 0.01 > traj_data(i,1)
            index = i;
        end
    end

    al = traj_data(index,8:10)';
    vl = traj_data(index,5:7)';

    vl=vl+dt*al;
    xl=xl+dt*vl;

    % get motor speeds form the controller
    omega=quadrotor_controller(s,xl,vl,0,para,1,10);
    
    %record the speeds
    omegaHis(:,t)=omega;
    
    %send speeds of four motors to quadrotor and get its state
    s=quadrotor_kinematics(s,omega,para,dt);

    real_traj(:,t) = s(1:3);
    ref_traj(:,t) = xl;
    
    %recodrd the position of quadrotor at time t/loop*stime
    % xyHis(:,:,t+1)=[xl s(1:3)];
    
    % waitbar(t/loop,hwait,'simulating...');
end
tTotal = toc(tStart);

% close(hwait);
% %show the animation of the flight process
% figure(1)
% plotHis3(xyHis,dt,-1,200)
% axis equal
% grid on
% 
% %show changes in motor speeds during the flight
% figure(2)
% plot(omegaHis')
% grid on
