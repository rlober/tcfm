classdef QpController < handle
    %QP_CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        R; %robot reference
        q;
        qd;
        t;
        tasks;
        constraints;
        E;
        f;
        Q;
        p;
        G;
        h;
        qp_options;
        using_constraints;
        torque_limit;
    end
    
    methods
        function obj = QpController(using_constraints, torque_limit)
            global robot;
            obj.R = robot;
            obj.torque_limit = torque_limit;
            obj.using_constraints = using_constraints;
            obj.qp_options = optimset('Algorithm','interior-point-convex', 'Display', 'off');

%             obj.tasks = {EEPoseTask(obj.R, 1.0, 10.0, 0.2)};
%             obj.tasks = {EETask(obj.R, 1.0, 10.0, 0.2)};
%             obj.tasks = {PostureTask(obj.R, 1.0, 10.0, 0.2)};
            obj.tasks = {EETask(obj.R, 1.0, 10.0, 0.2), PostureTask(obj.R, 0.001, 10.0, 0.2)};

%             obj.constraints = {TorqueConstraint(obj.R, -obj.torque_limit, obj.torque_limit)};
%             obj.constraints = {JointPositionConstraint(obj.R, obj.R.qlim(:,1), obj.R.qlim(:,2))};
            obj.constraints = {TorqueConstraint(obj.R, -obj.torque_limit, obj.torque_limit), JointPositionConstraint(obj.R, obj.R.qlim(:,1), obj.R.qlim(:,2))};
        end
        
        function tau = zero_torque(obj, t, q, qd)
           tau = zeros(obj.R.n, 1); 
        end
        
        function tau = compute_tau(obj, t, q, qd)
            obj.update(t, q, qd)
            
            if obj.using_constraints
                obj.update_constraints(t, q, qd);
                tau = obj.solve_qp();
            else
                tau = obj.solve_unconstrained_qp();
            end
        end
        
        function update(obj, t, q, qd)
            obj.q = q;
            obj.qd = qd;
            obj.t = t;
            
            obj.E = [];
            obj.f = [];
            for i = 1:size(obj.tasks, 1)
                obj.tasks{i}.update(t, q, qd);
                obj.E = [obj.E; sqrt(obj.tasks{i}.weight)*obj.tasks{i}.E];
                obj.f = [obj.f; sqrt(obj.tasks{i}.weight)*obj.tasks{i}.f];
            end
            reg_weight = 0.0001;
            obj.E = [obj.E; sqrt(reg_weight)*eye(obj.R.n)];
            obj.f = [obj.f; sqrt(reg_weight)*zeros(obj.R.n,1)];
             
            obj.Q = (obj.E')*obj.E;
            obj.p = -(obj.E'*obj.f)';
        end
        
        function update_constraints(obj, t, q, qd)
            for i = 1:size(obj.constraints)
               obj.constraints{i}.update(t, q, qd);
               obj.G = [obj.G; obj.constraints{i}.G];
               obj.h = [obj.h; obj.constraints{i}.h];
            end
        end
        
        function tau = solve_unconstrained_qp(obj)
           tau = pinv(obj.E)*obj.f;
        end
        
        function tau = solve_qp(obj)

           [tau,j_val,EXITFLAG] = quadprog(obj.Q,obj.p,obj.G,obj.h, [],[],[],[],[], obj.qp_options);
%            [tau,j_val,EXITFLAG] = quadprog(obj.Q,obj.p,[],[], [],[],[],[],[], obj.qp_options);
             if EXITFLAG == 0
                disp('Maximum number of iterations exceeded.')
             elseif EXITFLAG == -2
                disp('No feasible point found.')
                tau = [];
             elseif EXITFLAG == -3
                disp('Problem is unbounded.')
             elseif EXITFLAG == -6
                 disp('Non-convex problem detected.')
             end      
        end 
    end
end

