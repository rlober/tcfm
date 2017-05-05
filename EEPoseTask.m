classdef EEPoseTask < Task
    %EEPOSETASK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function obj = EEPoseTask(robot, weight, kp, kd)
            obj = obj@Task(robot, weight, kp, kd);
        end
        
        function J = get_jacobian(obj, q)
            J = obj.R.jacob0(q);
        end
        
        function dJqd = get_dJdq(obj, q, qd)
           dJqd = obj.R.jacob_dot(q, qd); 
        end
        
        function acc_des = get_desired_acc(obj, t, q, qd)
           acc_ref = zeros(6,1);
           vel_ref = zeros(6,1);
           pos_ref = [1 0 0 0.4; 0 1 0 -0.5; 0 0 1 0.4; 0 0 0 1];
           
           pos_real = obj.R.fkine(q);
           vel_real = (obj.R.jacob0(q)*qd');
           
           pos_err = pos_real - pos_ref;
           pos_rot_err = tr2rpy(pos_err)';
           pos_trl_err = transl(pos_err);
           vel_err = vel_real - vel_ref;
           acc_des = acc_ref - (obj.kp*[pos_trl_err; pos_rot_err] + obj.kd*vel_err);
        end
    end
    
end

