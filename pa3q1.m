%% PART 1
l = [1, 1]';
theta = [pi/4, pi/4]';

[pos, J] = evalRobot2D(l, theta);

disp part_1;
disp(pos);
disp(J);

%% PART 2

% Finite difference approximation for the given problem is perfect at small
% pertubations of h. Hence if we can handle small differences in output,
% and exactness is not required, we can approximate faster then analytical.

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

clear;
