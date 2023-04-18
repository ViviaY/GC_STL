traj_data = readtable("C:\Users\Beren\Desktop\STL\STLPlanning-master\STLPlanning-master\stlcg-1_3d.csv");
% traj_data = readtable("C:\Users\Beren\Downloads\STL_yt\yt\Ztunnel.csv");
traj_data = table2array(traj_data);
traj_data(:,1) = [];
% rho = traj_data(1,1);
% traj_data(:,1) = [];
plot3(traj_data(:,2),traj_data(:,3),traj_data(:,4));
% traj_data = discretizeTraj(traj_data);