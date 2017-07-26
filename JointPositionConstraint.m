classdef JointPositionConstraint < Constraint
    %TORQUECONSTRAINT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        t_old;
    end
    
    methods
        function obj = JointPositionConstraint(robot, lb, ub)
            obj = obj@Constraint(robot, lb, ub);
            obj.t_old = 0;
        end
        
        function update(obj, t, q, qd)
            
            dt = 0.2;
%             dt = 0.001;
            Minv = inv(obj.R.inertia(q));
            n = obj.R.coriolis(q, qd) * qd';
            g = obj.R.gravload(q)';
            Minvgn = Minv*(n - g);
            pred = (q' + dt*qd');
%             disp(pred)
            
            
            
            global use_reduced;
            if use_reduced
                obj.G = [Minv; -1*Minv];
                obj.h = [(2 / (dt^2))*(obj.ub - pred) + Minvgn; -1*((2 / (dt^2))*(obj.lb - pred) + Minvgn)];
            else
                obj.G = [eye(6), zeros(6,6); -1*eye(6), zeros(6,6)];
                obj.h = (2 / (dt^2))*[(obj.ub - pred); -1*(obj.lb - pred)];            
            end
            
            obj.t_old = t;
            
        end
        
    end
    
end

