function [theta] = invKin2D(l, theta0, pos, n, mode)

if (mode == 0) % Broyden's method

xk = theta0;
Bk = evalRobot2D(l, xk);
for k = 1:n
    % Update x_next
    [curr_pos, ~] = evalRobot2D(l, xk);
    fx = curr_pos - pos;
    disp(k);
    disp(fx);
    sk = -Bk\fx;
    x_next = xk + sk;

    % Update B_next
    [next_pos, ~] = evalRobot2D(l, x_next);
    f_next = next_pos - pos;
    yk = f_next - fx;
    B_next = Bk + ((yk - (Bk * sk)) * sk')/(sk' * sk);

    % update all "next" to "curr"
    Bk = B_next;
    xk = x_next;
end

theta = xk;

elseif (mode == 1) % Newton's method

xk = theta0; % Initial guess
tolerance = 0.001;
for k = 0:n
    [curr_pos, Jk] = evalRobot2D(l, xk);
    fx = curr_pos - pos;
    sk = -Jk\fx;
    xk = xk + sk;
end

theta = xk;

end


end
