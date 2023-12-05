% Global State
l = [0.8; 0.7];
mode = 1;
iters = 50;

% Intial Conditions
theta0 = [0.2259; 0.1707; 0.2277];
theta = invKin3D(l, theta0, [1; sqrt(2)/2; 0], iters, mode);
[pos, ~] = evalRobot3D(l, theta);
pos = pos';

disp(pos);

% Plot for iter
plotRobot3D(l, theta);
pause;
clf;

%% Error State Version

% Get a new set of angles
end_pos = -pos;
x_end_pos = linspace(pos(1), end_pos(1), 100);
y_end_pos = linspace(pos(2), end_pos(2), 100);
z_end_pos = linspace(pos(3), end_pos(3), 100);
end_pos_path = [x_end_pos; y_end_pos; z_end_pos];

max_cn = 0;
for k = 1:100
    theta = invKin3D(l, theta, end_pos_path(1:3, k), iters, mode);

    plotRobot3D(l, theta);
    clf;

    % Solve for condition number
    [~, J] = evalRobot3D(l, theta);
    J_inv = inv(J);
    cn = norm(J) * norm(J_inv);

    max_cn = max(cn, max_cn);
end

% Find worst conditioning
disp(max_cn);


%% ANSWER

% Conditioning becomes an issue as the robot eases towards the center
% platform. (upwards of 300 in some cases).
%
% There are a few solutions to this:
% - We can plan a path around the robot that ends in the correct position.
% - We can force big rotations to occur first, then smaller ones second and
%   even smaller third. Ex. Move the base, then the elbow, then the wrist.
% - We can take the nearest radial path, meaning we can define a minimum
%   domain that the robot is allowed to operate near itself, and do a
%   collision check to determine if it will hit itself. If it does, we plan
%   a path around itself using A* or some similarly appropriate path
%   planning algorithm. If we want manual control, as stated in the lab, we
%   could always manually implement bezier's, though that would be a pain.
