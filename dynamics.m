function dydt = dynamics(t, y, use_friction)
global robot;
global controller;
global stop_integration;
global torques;
global torque_times;
%DYNAMICS Summary of this function goes here
%   Detailed explanation goes here
    disp('t = ')
    disp(t)
    n_dof = robot.n;
    q = y(1:n_dof,:)';
    qd = y(n_dof+1:end, 1)';
    tau = controller.compute_tau(t, q, qd);
    if max(size(tau)) == 0
        dydt = zeros(size(y,1),1);
        disp('Stopping integration');
        stop_integration = true;
    else
        torques = [torques; tau'];
        torque_times = [torque_times; t];
        hits_lb = q <= robot.qlim(:,1)';
        if sum(hits_lb) > 0
            for i = 1:n_dof
                if hits_lb(i)
%                     q(i) = robot.qlim(i,1);
                    fprintf('Joint %i hitting its lower bound %2.2d\n', i, robot.qlim(i,1))
                end
            end
        end
        hits_ub = q >= robot.qlim(:,2)';
        if sum(hits_ub) > 0
            for i = 1:n_dof
                if hits_ub(i)
%                     q(i) = robot.qlim(i,2);
                    fprintf('Joint %i hitting its upper bound %d\n', i, robot.qlim(i,2))
                end
            end
        end
        %     tau = controller.zero_torque(t, q, qd);
        
        %     disp('tau = ')
        %     disp(tau')
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
        dydt(n_dof+1:end, :) = Minv * (tau - f - n - g);
    end
end

