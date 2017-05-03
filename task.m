function tau = task( robot, q, qd )
%TASK Summary of this function goes here
%   Detailed explanation goes here
    
    J = robot.jacob0(q);
    Minv = inv(robot.inertia(q));
    n = robot.coriolis(q, qd) * qd';
    g = robot.gravload(q)';
    
    dJdq = robot.jacob_dot(q, qd);
    acc_des = zeros(robot.n, 1);
    E = J*Minv;
    f = acc_des - dJdq - J*Minv*(n-g);
    
    tau = pinv(E)*f;

end

