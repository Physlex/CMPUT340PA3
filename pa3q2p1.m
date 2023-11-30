%% PART 1

disp part_1;

l = [0.8; 0.7];

% A
theta = [0; pi/2; 0];
plotRobot3D(l, theta);

[~, J] = evalRobot3D(l, theta);
J_inv = inv(J);
cn = norm(J) * norm(J_inv);

% cn of A is 2.4915 (non-singular)
% the angle is theta, as above.

% disp A;
% disp(cn);

pause;
clf;

% B
theta = [1; 0; 0];
plotRobot3D(l, theta);

[~, J] = evalRobot3D(l, theta);
J_inv = inv(J);
cn = norm(J) * norm(J_inv);

% cn of B is Inf (Singular due to straightness of arms)
% I believe this occurs because of gimble lock in general cases,
% but for this case specifically it's what happens whenever the arm is perfectly straight.
% Another one that causes this is [0; 0; 0], or any other instance of [x; 0; 0]
% the angle is theta, as above.

% disp B;
% disp(cn);

pause;
clf;
clear;