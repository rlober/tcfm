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
%             dt = t - obj.t_old;
%             if dt < 0.001 
%                 dt = 0.001;
%             end
            dt = 0.2;
            Minv = inv(obj.R.inertia(q));
            n = obj.R.coriolis(q, qd) * qd';
            g = obj.R.gravload(q)';
            Minvgn = Minv*(n - g);
            pred = (q' + dt*qd');
            
            obj.G = [Minv; -1*Minv];
            obj.h = [(2 / (dt^2))*(obj.ub - pred) + Minvgn; -1*((2 / (dt^2))*(obj.lb - pred) + Minvgn)];
            obj.t_old = t;
        end
        
    end
    
end

