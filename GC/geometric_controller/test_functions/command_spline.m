function desired = command_spline(t, spline_traj)

for i = 1:(size(spline_traj,1)-1)
    if t >= spline_traj(i,1)
        desired.x = [spline_traj(i,2) spline_traj(i,3) spline_traj(i,4)]';
        % desired.v = [spline_traj(i,5) spline_traj(i,6) spline_traj(i,7)]';
        desired.v = ([spline_traj(i+1,2) spline_traj(i+1,3) spline_traj(i+1,4)]' - ...
            [spline_traj(i,2) spline_traj(i,3) spline_traj(i,4)]') / ...
            (spline_traj(i+1,1) - spline_traj(i,1));
        % desired.x_2dot = [spline_traj(i,8) spline_traj(i,9) spline_traj(i,10)]';
        % desired.v = [0,0,0]';
        desired.x_2dot = [0,0,0]';
        desired.x_3dot = [0,0,0]';
        desired.x_4dot = [0,0,0]';
        desired.b1 = [1,0,0]';
        desired.b1_dot = [0,0,0]';
        desired.b1_2dot = [0,0,0]';
    end
end