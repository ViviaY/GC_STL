%% RRT-MPC-Quadcopter
% Quadcopter global and local path planning with Rapidly-Exploring Random
% Tree search and nonlinear Model Predictive Control. 
%
% Created by:
%   Christos Vasileio
%   Cristian Meo
%   Francesco Stella
%   Joris Verhagen
%
% MIT License
%
% Created: April 2020

%% Start
close all; clc;
addpath('MPC_functions')

global r_0 t timestep
rng(0)

%% load data
traj_data = readtable("C:\Users\Beren\Downloads\STL_yt\yt\stlcg-1_3d_with_notB1.csv");
% traj_data = readtable("C:\Users\Beren\Desktop\STL\STL_rho_yt\STL_rho_yt\0.1-stlcg-3d.csv");
% traj_data = readtable("C:\Users\Beren\Downloads\STL_yt\yt\Ztunnel.csv");
% traj_data = readtable("C:\Users\Beren\Downloads\STL_yt\yt\Ltunnel.csv");
% traj_data = readtable("C:\Users\Beren\Desktop\STL\STL_rho_yt\STL_rho_yt\stlcg-1_3d_seg9_yt.csv");
% traj_data = readtable("C:\Users\Beren\Downloads\STL_yt\yt\zigzag.csv");
traj_data = table2array(traj_data);
traj_data(:,1) = [];
traj_data(:,1) = [];

x_n = traj_data(:,2)';
y_n = traj_data(:,3)';
z_n = traj_data(:,4)';

%% constants
Ts = 0.5;
 %number of obstacles
Nobs=3;

%% create controller

numStates = 12;
numOutputs = 6;
numControl = 4;

nlobj = nlmpc(numStates,numOutputs,numControl);
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 8;
nlobj.ControlHorizon = 3;

nlobj.Model.StateFcn = "droneDT";
nlobj.Model.IsContinuousTime = false;
nlobj.Model.NumberOfParameters = 1;

nlobj.Model.OutputFcn = @(x,u,Ts) x(1:numOutputs);

%% define constraints
nlobj.Weights.OutputVariables = [1 1 1 1 1 1]*5;
nlobj.Weights.ManipulatedVariablesRate = [1 1 1 1]*0.1;


%% initialization
x0 = [x_n(1) y_n(1) z_n(1) 0 0 0 0 0 0 0 0 0]';
u0 = zeros(numControl,1);

EKF = extendedKalmanFilter(@droneStateFcn,@droneMeasurementFcn);
EKF.State = x0;
uk = u0;

nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};

%% run on simulated data
duration = round(size(x_n,1)); %changed by Fra to make it always consistent with trajectory.
% duration = traj_data(end,1);
yref = [x_n' y_n' z_n' x_n'-x_n' x_n'-x_n' x_n'-x_n'];
        
y = x0(1:6);

ukHistory = zeros(numControl,round(duration/Ts));
xHistory = zeros(numStates,round(duration/Ts+1));
xHistory(:,1) = x0;

xktotal=[];
c=1;
tStart = tic;
for timestep = 1:(length(x_n))
    t=timestep;
    xk = correct(EKF,y);
    xktotal=[xktotal,xk];  
   %compute optimal control actions
   % for ii = 1:Nobs
        if  timestep>288 %sum(abs(xk(1:3) - r_0(ii,:)')) < 3 &&
            if c==1
            nlobj.Weights.OutputVariables = [1 1 1 1 1 1]*1.7;
            nlobj.Weights.ManipulatedVariablesRate = [1 1 1 1]*0.1;
            nlobj.PredictionHorizon = 6;
            nlobj.ControlHorizon = 1;
            c=c+1;
            end
            nlobj.Optimization.CustomIneqConFcn = "myIneqConFunction_pers";
            %break
        else
           nlobj.Optimization.CustomIneqConFcn = "myIneqConFunction_blank";
        end
  % end
  
    [uk,nloptions,info] = nlmpcmove(nlobj,xk,uk,yref(timestep:min(timestep+9,(length(x_n))),:),[],nloptions);
    info.ExitFlag
    ukHistory(:,timestep) = uk;
    % Predict prediction model states for the next iteration
    predict(EKF,[uk; Ts]);
    % Implement first optimal control move
    x = droneDT(xk,uk,Ts);
    % Generate sensor data
    y = x(1:numOutputs) + randn(numOutputs,1)*0.01;
    % Save plant states
    xHistory(:,timestep+1) = x;
    timestep;
    
end
tTotal = toc(tStart);


figure
plot3(xHistory(1,:),xHistory(2,:),xHistory(3,:),'-')
title('drone location')
hold on
plot3(x_n,y_n,z_n)
xlim([-10 10])
ylim([-10 10])
zlim([-10 10])
grid on

figure
e_x = xHistory(1:3,1:40) - [x_n' y_n' z_n']';
plot(vecnorm(e_x));

% subplot(1,2,2)
% plot(ukHistory(1,:))
% hold on;
% plot(ukHistory(2,:))
% plot(ukHistory(3,:))
% plot(ukHistory(4,:))
% title('control input')
% grid on

