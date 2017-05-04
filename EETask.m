classdef EETask < Task
    %EETASK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function obj = EETask(robot, weight, kp, kd)
            obj = obj@Task(robot, weight, kp, kd);
        end
        
        function J = get_jacobian(obj, q)
            Jtmp = obj.R.jacob0(q);
            J = [ones(3,obj.R.n); zeros(3,obj.R.n)] .* Jtmp;
        end
        
        function dJqd = get_dJdq(obj, q, qd)
           dJqd = [ones(3,1); zeros(3,1)] .* obj.R.jacob_dot(q, qd); 
        end
        
        function acc_des = get_desired_acc(obj, t, q, qd)
           acc_ref = zeros(3,1);
           vel_ref = zeros(3,1);
           pos_ref = [0.25; 0.25; 0.25];
           
           pos_real = transl(obj.R.tool);
           vel_real = (obj.R.jacob0(q)*qd');
           vel_real = vel_real(1:3,1);
           
           pos_err = pos_real - pos_ref;
           vel_err = vel_real - vel_ref;
           acc_des = acc_ref - (obj.kp*pos_err + obj.kd*vel_err);
           acc_des = [acc_des; zeros(3,1)];
        end
    end
    
end

