% Global State
l = [0.8; 0.7];
mode = 1;
iters = 1000;

% Intial Conditions
theta = invKin3D(l, rand(3,1), rand(3,1), iters, mode);
disp(theta);
[pos, J] = evalRobot3D(l, theta);
pos = pos';

%A
% Because the tests are randomized, you may need to run it with different
% global state to guarentee seeing the end effector want to reach beyond
% it's possible reach length.
%
% The reason newtons stops converging, is because solving the equations
% specified in the question 2 section requires solving for 2 seperate
% components of one equation. The desired position, and whatever position
% we can compute from just angles and length.
%
% This will converge to 0 only in situations where this equation can
% actually converge to 0. Because of how the equations are set up however,
% if the desired position is too far, then the length cannot scale the
% angle results enough to actually reach the desired position.
%
% Hence, we cannot converge on a solution.

% B
% The easiest way to deal with this is to clamp the vector components to a
% specific coordinate range. It would be like setting the domain of a
% funciton. To answer the part regarding singularities, the easiest way to
% deal with all of this at once is to just use quaternions, which have no
% singularities.

% Main Loop
for k = 0:20
    % Plot for iter
    plotRobot3D(l, theta);
    pause;
    clf;

    % Get a new set of angles
    pos(3) = pos(3) + (0.05);
    theta = invKin3D(l, rand(3, 1), pos, iters, mode);

    % Update the position for the next iter from the angles
    [pos, J] = evalRobot3D(l, theta);
    pos = pos';

    % Solve for condition number
    J_inv = inv(J);
    cn = norm(J) * norm(J_inv);

    % Display relavent results
    disp(pos);
    disp(cn);
end
