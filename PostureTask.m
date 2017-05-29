classdef PostureTask < Task
    %EETASK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function obj = PostureTask(robot, weight, kp, kd)
            obj = obj@Task(robot, weight, kp, kd);
            obj.acc_ref = zeros(obj.R.n,1);
            obj.vel_ref = zeros(obj.R.n,1);
            global qn;
            obj.pos_ref = qn';
        end
        
        function J = get_jacobian(obj, q)
            J = eye(size(q,2));
        end
        
        function dJdq = get_dJdq(obj, q, qd)
           dJdq = qd';
        end
        
        function acc_des = get_desired_acc(obj, t, q, qd)
           pos_real = q';
           vel_real = qd';
           
           pos_err = obj.pos_ref - pos_real;
           vel_err = obj.vel_ref - vel_real;
           acc_des = obj.acc_ref + obj.kp*pos_err + obj.kd*vel_err;
        end
        
        function n_dof = getTaskDof(obj)
            n_dof = obj.R.n;
        end
        
        function start_pos = getStartPosition(obj, q)
            start_pos = q;
        end
    end
    
end

