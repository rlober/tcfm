classdef Constraint < handle
    %CONSTRAINT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        G;
        h;
        lb;
        ub;
        R;
        n_dof;
    end
    
    methods
        function obj = Constraint(robot, lb, ub)
            obj.R = robot;
            obj.n_dof = obj.R.n;
            if max(size(lb)) == 1
                obj.lb = ones(obj.n_dof,1)*lb;
            elseif size(lb,2) == obj.n_dof
                obj.lb = lb';
            else
                obj.lb = lb;
            end
            
            if max(size(ub)) == 1
                obj.ub = ones(obj.n_dof,1)*ub;
            elseif size(ub,2) == obj.n_dof
                obj.ub = ub';
            else
                obj.ub = ub;
            end
            
        end
    end
    
end

