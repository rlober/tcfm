classdef PostureTask < Task
    %EETASK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function obj = PostureTask(robot, weight, kp, kd)
            obj = obj@Task(robot, weight, kp, kd);
        end
        
        function J = get_jacobian(obj, q)
            J = eye(size(q,2));
        end
        
        function dJdq = get_dJdq(obj, q, qd)
           dJdq = qd';
        end
        
        function acc_des = get_desired_acc(obj, t, q, qd)
           acc_ref = zeros(obj.R.n,1);
           vel_ref = zeros(obj.R.n,1);
           pos_ref = [0,1.57079632679490,-1.57079632679490,0,0,0]';
           
           pos_real = q';
           vel_real = qd';
           
           pos_err = pos_ref - pos_real;
           vel_err = vel_real - vel_ref;
           acc_des = acc_ref + obj.kp*pos_err - obj.kd*vel_err;
        end
    end
    
end

