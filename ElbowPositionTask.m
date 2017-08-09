classdef ElbowPositionTask < Task
    %EETASK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        sub_robot;
        n_links;
    end
    
    methods
        function obj = ElbowPositionTask(robot, weight, kp, kd)
            obj = obj@Task(robot, weight, kp, kd);
            obj.n_links = 3;
            obj.sub_robot = SerialLink(obj.R.links(1:obj.n_links));
            obj.acc_ref = zeros(3,1);
            obj.vel_ref = zeros(3,1);
            global qn;
            obj.pos_ref = obj.sub_robot.fkine(qn(1:obj.n_links));
            obj.pos_ref = obj.pos_ref(1:3,4);
        end
        
        function J = get_jacobian(obj, q)
            Jtmp = obj.sub_robot.jacob0(q(1:obj.n_links));
            J = Jtmp(1:3,:);
        end
        
        function dJdq = get_dJdq(obj, q, qd)
           dJdq_tmp = obj.sub_robot.jacob_dot(q(1:obj.n_links), qd(1:obj.n_links)); 
           dJdq = dJdq_tmp(1:3,:);
        end
        
        function acc_des = get_desired_acc(obj, t, q, qd)
           q = q(1:obj.n_links);
           qd = qd(1:obj.n_links);
           pos_real = transl(obj.sub_robot.fkine(q));
           vel_real = (obj.sub_robot.jacob0(q)*qd');
           vel_real = vel_real(1:3,1);
           
           pos_err = obj.pos_ref - pos_real;
           vel_err = obj.vel_ref - vel_real;
           acc_des = obj.acc_ref + obj.kp*pos_err + obj.kd*vel_err;
        end
        
        function update(obj, t, q, qd)
            if obj.using_trajectory
                obj.minJerkTrajectory(t, q);
            end
            
            global use_reduced;
            if ~use_reduced
                M_big = obj.R.inertia(q);
                n_big = obj.R.coriolis(q, qd) * qd';
                g_big = obj.R.gravload(q)';
            end
            
            q = q(1:obj.n_links);
            qd = qd(1:obj.n_links);
            
            obj.J = obj.get_jacobian(q);
            dJdq = obj.get_dJdq(q, qd);
            obj.acc_des = obj.get_desired_acc(t, q, qd);
            
            obj.desired_accelerations = [obj.desired_accelerations; {t, obj.acc_des}];
            obj.desired_acceleration_norms = [obj.desired_acceleration_norms; {t, norm(obj.acc_des)}];
            
            obj.references = [obj.references; {t, obj.pos_ref, obj.vel_ref, obj.acc_ref}];

            
            if use_reduced
                Minv = inv(obj.sub_robot.inertia(q));
                n = obj.sub_robot.coriolis(q, qd) * qd';
                g = obj.sub_robot.gravload(q)';
                obj.E = [obj.J*Minv, zeros(3,(obj.R.n - obj.sub_robot.n))];
                obj.f = obj.acc_des - dJdq - obj.J*Minv*(n-g);
                obj.tau = pinv(obj.E)*obj.f;
            else
                obj.E = [obj.J, zeros(3,(obj.R.n - obj.sub_robot.n)), zeros(3,6)];
                obj.f = obj.acc_des - dJdq;
               
                A = [-M_big, eye(6)];
                b = n_big + g_big;
%                 obj.tau = lsqlin(obj.E, obj.f, A, b, [], [], [], [], [], obj.qp_options);
                obj.tau = CLS(obj.E, obj.f, A, b);
            end
        end
        
        function n_dof = getTaskDof(obj)
            n_dof = 3;
        end
        
        function start_pos = getStartPosition(obj, q)
            pos_real = obj.sub_robot.fkine(q(1:obj.n_links));
            start_pos = pos_real(1:3,4);
        end
    end
    
end

