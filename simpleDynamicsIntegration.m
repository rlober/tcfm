function [ t, y ] = simpleDynamicsIntegration(  use_friction, tspan, y0 )
%SIMPLEDYNAMICSINTEGRATION Summary of this function goes here
%   Detailed explanation goes here
global robot;
global controller;
dt = tspan(1)-tspan(2);
y = ones(size(tspan,2),size(y0,1));
y(1,:) = y0';
for i = 1:size(tspan,2)-1
    t = tspan(i);
    dydt = dynamics(t, y(i,:)', use_friction);
    y(i+1,:) = y(i,:)' + dydt * dt;
end

end

