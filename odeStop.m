function [ value, isterminal, direction ] = odeStop( t, y )
%ODESTOP Summary of this function goes here
%   Detailed explanation goes here
global stop_integration;

if stop_integration
    value = y; % detect y-1/2 = 0
    isterminal = 1; % stop the integration
    direction = 0; % negative direction
end

end

