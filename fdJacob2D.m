function [J] = fdJacob2D(l, theta, h)

% theta wrt theta(1)
col1 = ( evalRobot2D(l, theta + [h; 0]) - evalRobot2D(l, theta - [h; 0]) ) / (2 * h);
% theta wrt theta(2)
col2 = ( evalRobot2D(l, theta + [0; h]) - evalRobot2D(l, theta - [0; h]) ) / (2 * h);

J = [col1 col2];

end
