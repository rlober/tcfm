classdef Task < handle
    %TASK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        R; %robot
        weight;
        kp;
        kd;
        J;
        E;
        f;
        acc_des;
        tau;
        
    end
    
    methods
        function obj = Task(robot, weight, kp, kd)
            obj.R = robot;
            obj.weight = weight;
            obj.kp = kp;
            obj.kd = 2*sqrt(kp);
        end
        
        
        function update(obj, t, q, qd)
            obj.J = obj.get_jacobian(q);
            dJdq = obj.get_dJdq(q, qd);
            obj.acc_des = obj.get_desired_acc(t, q, qd);
            
%             disp('acc_des = ')
%             disp(obj.acc_des')
            
            Minv = inv(obj.R.inertia(q));
            n = obj.R.coriolis(q, qd) * qd';
            g = obj.R.gravload(q)';

            obj.E = obj.J*Minv;
            obj.f = obj.acc_des - dJdq - obj.J*Minv*(n-g);

            obj.tau = pinv(obj.E)*obj.f; 
        end
        
        
    end
    
end

