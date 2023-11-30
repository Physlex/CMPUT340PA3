function [theta] = invKin3D(l,theta0,pos,n,mode)

if (mode == 0) % Broyden's method

xk = theta0;
[~, Bk] = evalRobot3D(l, xk);
for k = 1:n
    % Update x_next
    [curr_pos, ~] = evalRobot3D(l, xk);
    curr_pos = curr_pos';
    fx = curr_pos - pos;
    sk = -Bk\fx;
    x_next = xk + sk;

    % Update B_next
    [next_pos, ~] = evalRobot3D(l, x_next);
    next_pos = next_pos';
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
for k = 0:n
    [curr_pos, Jk] = evalRobot3D(l, xk);
    curr_pos = curr_pos';
    fx = curr_pos - pos;
    sk = -Jk\fx;
    xk = xk + sk;
end

theta = xk;

end

end
