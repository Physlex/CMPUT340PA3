function [pos, J] = evalRobot2D(l, theta)
% l holds [l_1, l_2]. The lengths of the links
% theta holds [theta_1, theta_2]. The angles of the links
%
% We wish to return a vector of positions [x, y] and
% the jacobian of pos wrt theta. J must be analytical.
%
% If we solve the initial

x = l(1) * cos(theta(1)) + l(2) * cos(theta(1) + theta(2));
y = l(1) * sin(theta(1)) + l(2) * sin(theta(1) + theta(2));

pos = [x, y]';

x1 = -l(1) * sin(theta(1)) - l(2) * sin(theta(1) + theta(2));
x2 = -l(2) * sin(theta(1) + theta(2));

x3 = l(1) * cos(theta(1)) + l(2) * cos(theta(1) + theta(2));
x4 = l(2) * cos(theta(1) + theta(2));

J = [x1 x2; x3 x4];

end
