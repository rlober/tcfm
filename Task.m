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
        acc_ref;
        vel_ref;
        pos_ref;
        using_trajectory;
        using_spline_trajectory;
        pos_des;
        start_pos;
        alpha;
        pointToPointDuration;
        first_traj_call;
        t0;
        references;
        desired_accelerations;
        desired_acceleration_norms;
        max_vel;
        qp_options;
        real_pos;
        spline_traj;
    end
    
    methods
        function obj = Task(robot, weight, kp, kd)
            obj.R = robot;
            obj.weight = weight;
            obj.kp = kp;
            obj.kd = 2*sqrt(kp);
            obj.using_trajectory = false;
            obj.references = {};
            obj.desired_accelerations = {};
            obj.desired_acceleration_norms = {};
            obj.real_pos=[];
            obj.max_vel = 0.2;
            obj.qp_options = optimset('Algorithm','interior-point', 'Display', 'off');
            obj.pointToPointDuration = 0;
        end
        
        
        function update(obj, t, q, qd)
            if obj.using_trajectory
                obj.minJerkTrajectory(t, q);
            end
            
            if obj.using_spline_trajectory
                [obj.pos_ref, obj.vel_ref, obj.acc_ref] = obj.spline_traj.get_references(t);
%                 disp('EE')
%                 disp(obj.pos_ref')
%                 disp(obj.vel_ref)
%                 disp(obj.acc_ref)
            end
            
            obj.J = obj.get_jacobian(q);
            dJdq = obj.get_dJdq(q, qd);
            obj.acc_des = obj.get_desired_acc(t, q, qd);
            
            obj.desired_accelerations = [obj.desired_accelerations; {t, obj.acc_des}];
            obj.desired_acceleration_norms = [obj.desired_acceleration_norms; {t, norm(obj.acc_des)}];

            obj.references = [obj.references; {t, obj.pos_ref, obj.vel_ref, obj.acc_ref}];
%             disp('acc_des = ')
%             disp(obj.acc_des')

            M = obj.R.inertia(q);
            n = obj.R.coriolis(q, qd) * qd';
            g = obj.R.gravload(q)';
                
            global use_reduced;
            if use_reduced
                Minv = inv(M);
                obj.E = obj.J*Minv;
                obj.f = obj.acc_des - dJdq - obj.J*Minv*(n-g);
                
                obj.tau = pinv(obj.E)*obj.f;
            else
                obj.E = [obj.J, zeros(size(obj.J,1),6)];
                obj.f = obj.acc_des - dJdq;
                A = [-M, eye(6)];
                b = n + g;
%                 obj.tau = lsqlin(obj.E, obj.f, A, b, [], [], [], [], [], obj.qp_options);
                obj.tau = CLS(obj.E, obj.f, A, b);
%                 disp(obj.tau')
            end
            
        end
        
        function setReferences(obj, pos, vel, acc)
           if max(size(pos))>0
               obj.pos_ref = pos;
           end
           
           if max(size(vel))>0
               obj.vel_ref = vel;
           end
           
           if max(size(acc))>0
               obj.acc_ref = acc;
           end
           
        end
        
        function setDesired(obj, pos)
           obj.using_trajectory = true;
           obj.pos_des = pos;
           obj.first_traj_call = true;
        end
        
        function setDesiredSpline(obj, times, waypoints)
           obj.using_spline_trajectory = true;
           obj.spline_traj = SplineTrajectory(times, waypoints);
           obj.first_traj_call = true;
        end
        
        function minJerkTrajectory(obj, t, q)
            
            if obj.first_traj_call
                obj.start_pos = obj.getStartPosition(q);
                obj.alpha = obj.pos_des - obj.start_pos;
                obj.pointToPointDuration = norm(obj.alpha) / obj.max_vel;
                obj.first_traj_call = false;
                obj.t0 = t;
            end
            
            if t <= obj.pointToPointDuration
                beta = (t - obj.t0) / obj.pointToPointDuration;
                n_dof = obj.getTaskDof();
                obj.pos_ref = obj.start_pos  + obj.alpha * ( 10*(beta^3.0) - 15*(beta^4.0)  + 6*(beta^5.0)   );
                obj.vel_ref = zeros(n_dof,1) + obj.alpha * ( 30*(beta^2.0) - 60*(beta^3.0)  + 30*(beta^4.0)  );
                obj.acc_ref = zeros(n_dof,1) + obj.alpha * ( 60*(beta^1.0) - 180*(beta^2.0) + 120*(beta^3.0) );
            else
                obj.using_trajectory = false;
            end
            
        end
            
        
    end
    
end

