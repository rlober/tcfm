classdef QpController < handle
    %QP_CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        R; %robot reference
        q;
        qd;
        t;
        tasks;
        E;
        f;
    end
    
    methods
        function obj = QpController(robot)
            obj.R = robot;
            
            obj.tasks = {EEPoseTask(obj.R, 1.0, 10.0, 0.2)};
        end
        
        function tau = zero_torque(obj, t, q, qd)
           tau = zeros(obj.R.n, 1); 
        end
        
        function tau = compute_tau(obj, t, q, qd)
            obj.update(t, q, qd)
            tau = obj.solve_qp;
        end
        
        function update(obj, t, q, qd)
            obj.q = q;
            obj.qd = qd;
            obj.t = t;
            
            obj.E = [];
            obj.f = [];
            for i = 1:size(obj.tasks, 1)
                obj.tasks{i}.update(t, q, qd);
                obj.E = [obj.E; obj.tasks{i}.weight*obj.tasks{i}.E];
                obj.f = [obj.f; obj.tasks{i}.weight*obj.tasks{i}.f];
            end
        end
        
        function tau = solve_qp(obj)
           tau = pinv(obj.E)*obj.f;
        end
       
        
        
    end
    
end

