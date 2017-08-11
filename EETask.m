classdef EETask < Task
    %EETASK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function obj = EETask(robot, weight, kp, kd)
            obj = obj@Task(robot, weight, kp, kd);
            obj.acc_ref = zeros(3,1);
            obj.vel_ref = zeros(3,1);
            global qn;
            obj.pos_ref = obj.R.fkine(qn);
            obj.pos_ref = obj.pos_ref(1:3,4);
        end
        
        function J = get_jacobian(obj, q)
            Jtmp = obj.R.jacob0(q);
            J = Jtmp(1:3,:);
        end
        
        function dJdq = get_dJdq(obj, q, qd)
           dJdq_tmp = obj.R.jacob_dot(q, qd); 
           dJdq = dJdq_tmp(1:3,:);
        end
        
        function acc_des = get_desired_acc(obj, t, q, qd)
           pos_real = transl(obj.R.fkine(q));
           vel_real = (obj.R.jacob0(q)*qd');
           vel_real = vel_real(1:3,1);
           
           obj.real_pos = [obj.real_pos; pos_real'];
           
           pos_err = obj.pos_ref - pos_real;
           vel_err = obj.vel_ref - vel_real;
           acc_des = obj.acc_ref + obj.kp*pos_err + obj.kd*vel_err;
        end
        
        function n_dof = getTaskDof(obj)
            n_dof = 3;
        end
        
        function start_pos = getStartPosition(obj, q)
            pos_real = obj.R.fkine(q);
            start_pos = pos_real(1:3,4);
        end
    end
    
end

