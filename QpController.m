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
        optima;
        compute_metrics;
        Es;
        fs;
        metric_data;
        t_old;
        metric_compute_dt;
    end
    
    methods
        function obj = QpController(tasks, constraints, using_constraints, compute_metrics, torque_limit)
            global robot;
            obj.R = robot;
            obj.torque_limit = torque_limit;
            obj.using_constraints = using_constraints;
            obj.compute_metrics = compute_metrics;
            obj.metric_data = {};
            obj.qp_options = optimset('Algorithm','interior-point-convex', 'Display', 'off');
            obj.tasks = tasks;
            obj.constraints = constraints;
            obj.t_old = 0.0;
            obj.metric_compute_dt = 0.1;
        end
        
        function tau = zero_torque(obj, t, q, qd)
           tau = zeros(obj.R.n, 1); 
        end
        
        function tau = compute_tau(obj, t, q, qd)
            obj.update(t, q, qd)
            
            if obj.using_constraints
                obj.update_constraints(t, q, qd);
                if obj.compute_metrics
                    if (t-obj.t_old) > obj.metric_compute_dt
                        disp('Computing objective compatibility and feasibility metrics');
                        n_objectives = size(obj.Es,2);
                        x_star = obj.solve_unconstrained_qp();
                        [obj_el, con_el] = computeConstraintAndObjectiveEllipses(obj.optima,obj.G,obj.h,0);
                        
                        [ compatibility_metrics, feasibility_metrics ] = computeMetrics( obj.Es, obj.fs, obj.optima, x_star, obj_el, con_el);
                        
                        obj.metric_data = [obj.metric_data; {t, compatibility_metrics, feasibility_metrics, obj_el, con_el, n_objectives}];
                        obj.t_old = t;
                    end
                end
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
             obj.Es = {};
             obj.fs = {};
             obj.optima = [];
            for i = 1:size(obj.tasks, 2)
                obj.tasks{i}.update(t, q, qd);
                
                if obj.compute_metrics
                  
                   obj.Es = [obj.Es, obj.tasks{i}.E]; 
                   obj.fs = [obj.fs, obj.tasks{i}.f];
                   obj.optima = [obj.optima, obj.tasks{i}.tau];
                end
                
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
            obj.G = [];
            obj.h = [];
            for i = 1:size(obj.constraints,2)
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

