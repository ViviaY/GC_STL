load('C:\Users\Beren\Downloads\quadrotorTrackingControl-master\quadrotorTrackingControl-master\0.1_stlcg-3d_mpc_workspace.mat')
ex = vecnorm(real_traj - ref_traj);
plot(ex);
hold on;
load('C:\Users\Beren\Desktop\STL\uav_geometric_control-2\uav_geometric_control-master\matlab\0.1-stlcg-3d_workspace.mat')
plot(vecnorm(e.x));
xlabel("time step");
ylabel("||e_p||");
legend("MPC","GC");
title("STLcg task position error");