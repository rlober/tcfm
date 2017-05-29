classdef EEPoseTask < Task
    %EEPOSETASK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function obj = EEPoseTask(robot, weight, kp, kd)
            obj = obj@Task(robot, weight, kp, kd);
            obj.acc_ref = zeros(6,1);
            obj.vel_ref = zeros(6,1);
            global qn;
            obj.pos_ref = obj.R.fkine(qn);
        end
        
        function J = get_jacobian(obj, q)
            J = obj.R.jacob0(q);
        end
        
        function dJqd = get_dJdq(obj, q, qd)
           dJqd = obj.R.jacob_dot(q, qd); 
        end
        
        function acc_des = get_desired_acc(obj, t, q, qd)
           pos_real = obj.R.fkine(q);
           [dP, dR] = PoseError(obj.pos_ref, pos_real);
           
           vel_real = (obj.R.jacob0(q)*qd');
           vel_err = obj.vel_ref - vel_real;
           
           acc_des = obj.acc_ref + obj.kp*[dP; dR] + obj.kd*vel_err;
        end
        
        function n_dof = getTaskDof(obj)
            n_dof = 4;
        end
        
        function start_pos = getStartPosition(obj, q)
            pos_real = obj.R.fkine(q);
            [theta,v] = tr2angvec(pos_real(1:3,1:3));
            start_pos = pos_real
        end
    end
    
end

