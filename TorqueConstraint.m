classdef TorqueConstraint < Constraint
    %TORQUECONSTRAINT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        function obj = TorqueConstraint(robot, lb, ub)
            obj = obj@Constraint(robot, lb, ub);
         
            global use_reduced;
            if use_reduced
                obj.G = [eye(obj.n_dof); -1*eye(obj.n_dof)];
                obj.h = [obj.ub; -1*obj.lb];
            else
                obj.G = [zeros(6,6), eye(obj.n_dof); zeros(6,6), -1*eye(obj.n_dof)];
                obj.h = [ obj.ub; -1*obj.lb];
            end
        end
        
        function update(obj, t, q, qd)
%             Do nothing
        end
        
    end
    
end

