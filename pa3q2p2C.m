% Global State
l = [0.8; 0.7];
mode = 1;
iters = 1000;

% Intial Conditions
theta = invKin3D(l, rand(3,1), rand(3,1), iters, mode);
[pos, ~] = evalRobot3D(l, theta);
pos = pos';

% Again, because it's random we can't really say anything specific. But,
% the updating function will swap itself around quickly. The way to solve
% this solution is to plan a path by swivel, linearly interpolating a
% longer path around the arm rather than having it attempt to cross
% directly to that point. To accomplish this, we can have intermediary
% points it moves to before it executes a diametrically opposed movement,
% or we could move the largest angles at a time first so that it swivels
% the whole body, then the largest arm segment, then the next largest and
% so on. These all prevent swiveling directly across the ceneter of the
% axis.

%% Error State Version

% Plot for iter
plotRobot3D(l, theta);
pause;
clf;

% Get a new set of angles
pos = -pos;
theta = invKin3D(l, rand(3, 1), pos, iters, mode);

% Solve for condition number
J_inv = inv(J);
cn = norm(J) * norm(J_inv);

% Display relavent results
disp(pos);
disp(cn);

%% Path Planned Version


% Plot for iter
plotRobot3D(l, theta);
pause;
clf;

% Get a new set of angles
pos = -pos;
theta = invKin3D(l, rand(3, 1), pos, iters, mode);

% Solve for condition number
J_inv = inv(J);
cn = norm(J) * norm(J_inv);

% Display relavent results
disp(pos);
disp(cn);
