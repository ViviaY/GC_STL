function desired = command_line(t)

% height = 1;
w = 2 * pi / 10;
desired.x = [0,0,0]';
desired.v = [0,0,0]';

% if t < 10
%     desired.x = [0, - 5 + 0.5 * t, 0]';
%     desired.v = [0, 0.5, 0]';
% else
%     desired.x = [0, sin(w * (t - 10)), cos(w * (t - 10)) - 1]';
%     desired.v = w * [0, cos(w * (t - 10)), -sin(w * (t - 10))]';
% end

% a = 0.5;
% desired.x = [0, 0.5 * a * t^2,0]';
% desired.v = [0, a * t,0]';
% desired.x_2dot = [0, a, 0]';
% desired.x_3dot = [0, 0, 0]';
% desired.x_4dot = [0, 0, 0]';

desired.x = [cos(w*t), 0,0]';
desired.v = w*[-sin(w*t), 0,0]';
desired.x_2dot = w^2*[-cos(w*t), 0, 0]';
desired.x_3dot = w^3*[sin(w*t), 0, 0]';
desired.x_4dot = w^4*[cos(w*t), 0, 0]';


% desired.b1 = [cos(w * t), sin(w * t), 0]';
% desired.b1_dot = w * [-sin(w * t), cos(w * t), 0]';
% desired.b1_2dot = w^2 * [-cos(w * t), -sin(w * t), 0]';

desired.b1 = [1,0,0]';
desired.b1_dot = [0,0,0]';
desired.b1_2dot = [0,0,0]';

end