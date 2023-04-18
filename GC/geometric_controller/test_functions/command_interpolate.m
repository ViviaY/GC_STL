function desired = command_interpolate(t, traj)


for i = 1:size(traj,1)
    if t >= traj(i,1)
        desired.x = [traj(i,2) traj(i,3) traj(i,4)]' + ...
            [traj(i,5) traj(i,6) traj(i,7)]' * (t - traj(i,1)) + ...
            1/2 * [traj(i,8) traj(i,9) traj(i,10)]' * (t - traj(i,1))^2;
        desired.v = [traj(i,5) traj(i,6) traj(i,7)]' + ...
            [traj(i,8) traj(i,9) traj(i,10)]' * (t - traj(i,1));
        desired.x_2dot = [traj(i,8) traj(i,9) traj(i,10)]';
        % desired.v = [0,0,0]';
        % desired.x_2dot = [0,0,0]';
        desired.x_3dot = [0,0,0]';
        desired.x_4dot = [0,0,0]';
        desired.b1 = [1,0,0]';
        desired.b1_dot = [0,0,0]';
        desired.b1_2dot = [0,0,0]';
    end
end