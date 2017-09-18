classdef SplineTrajectory < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        times;
        positions;
        last_pos;
        last_vel;
        last_acc;
        last_time;
        inner_spline_object;
        max_time;
        z_vec;
    end
    
    methods
        function obj = SplineTrajectory(times, positions)
           obj.times = times;
           obj.positions = positions;
           obj.last_time = 0.0;
%            create a clamped spline
%           times should be a row vector, and positions should be a matrix with each
%           column being a waypoint.
           n_dof = size(positions,1);
           z_vec = zeros(n_dof,1);
           obj.z_vec = z_vec;
           obj.last_vel = z_vec;
           obj.last_acc = z_vec;
           obj.inner_spline_object = spline(times, [z_vec positions z_vec]);
           
           obj.last_pos = obj.interpolate(obj.last_time);
           obj.max_time = times(end);
        end
        
        function new_pos = interpolate(obj, new_time)
           new_pos = ppval(obj.inner_spline_object, new_time); 
        end
        
        function [new_vel, new_acc] = derive_vel_and_acc(obj, new_time, new_pos)
            dt = new_time - obj.last_time;
            if dt == 0
                new_vel = obj.z_vec;
                new_acc = obj.z_vec;
            else
                new_vel = (new_pos - obj.last_pos)/dt;
                new_acc = (new_vel - obj.last_vel)/dt;
            end
        end
        
        function [pos, vel, acc] = get_references(obj, new_time)
            if new_time > obj.max_time
                pos = obj.last_pos;
                vel = obj.last_vel;
                acc = obj.last_acc;
            else
                pos = obj.interpolate(new_time);
                [vel, acc] = obj.derive_vel_and_acc(new_time, pos);
                obj.last_pos = pos;
                obj.last_vel = vel;
                obj.last_acc = acc;
                obj.last_time = new_time;
            end
        end
        
        
        
    end
    
end

