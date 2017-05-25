classdef TorqueConstraint < Constraint
    %TORQUECONSTRAINT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        function obj = TorqueConstraint(robot, lb, ub)
            obj = obj@Constraint(robot, lb, ub);
         
            obj.G = [eye(obj.n_dof); -1*eye(obj.n_dof)];
            obj.h = [obj.ub; -1*obj.lb];
        end
        
        function update(obj, t, q, qd)
%             Do nothing
        end
        
    end
    
end

