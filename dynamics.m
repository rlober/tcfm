function dydt = dynamics(t, y, robot, tau, use_friction)
%DYNAMICS Summary of this function goes here
%   Detailed explanation goes here
    n_dof = robot.n;
    q = y(1:n_dof,:)';
    qd = y(n_dof+1:end, 1)';
    M = robot.inertia(q);
    Minv = inv(M);
    n = robot.coriolis(q, qd) * qd';
    g = robot.gravload(q)';
    f = zeros(n_dof, 1);
    if use_friction
        f = robot.friction(qd)';
    end
    dydt = zeros(2*n_dof,1);
    dydt(1:n_dof,:) = qd';
    dydt(n_dof+1:end, :) = Minv * (tau + f - n - g);

end

