%% PART 1
l = [1, 1]';
theta = [pi/4, pi/4]';

[pos, J] = evalRobot2D(l, theta);

disp part_1;
disp(pos);
disp(J);

%% PART 2

% A
% Finite difference approximation for the given problem is perfect at small
% pertubations of h.

% B
% In general solutions however, we typically reserve non-analytical sols.
% for functions in which there is non-analytical answers, or the analytical
% answer is very hard to retrieve.

J0 = fdJacob2D(l, theta, 5);
J1 = fdJacob2D(l, theta, 0.001);
J2 = fdJacob2D(l, theta, 0.1);

disp part_2;
disp(J0);
disp(J1);
disp(J2);

%% PART 3 ^ 4

disp part_3;

% Robot Plotting
n = 100;

% A
% Broyden's method and newtons method both have the potential
% to send the robot angles to a value that fits a local root
% but does not satisfy the correct final position.
% It is my assumption that this occurs in situations where
% broyden and newtons method with inital guess of theta0
% would fail normally.
%
% Broyden's method is wildly innacurate compared to newtons.
% In part, this is because the jacobian is derived via approximation.
% Hence newtons, which has our hard-coded analytical jacobian, has more
% accuracy, dependent only on machine specification, on each iteration.
% This compares to Broydens, which has a convergent jacobian. Hence
% at some point the jacobian may "settle" too soon or late, and cause
% the solution of a root for nonlinear equation to be innacurate to
% a surprisingly large error.

% Set to newtons by default
mode = 1;

ls = [0.5; 0.5];
t = rand(2, 1);

clf;
plotRobot2D(ls, t);
hold off;

while (1)
    desired = ginput(1)';

    clf;
    plot(desired(1), desired(2), '*');
    hold on;
    plotRobot2D(ls, t, ':');

    % Solve and display the position
    t = invKin2D(ls, t, desired, n, mode);
    plotRobot2D(ls, t);
    hold off;
end
