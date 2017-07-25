function [ t, y ] = euler_1( fn_handle, tspan, y0 )
global stop_integration;
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
t = tspan';
h=t(2,1)-t(1,1);                                             % step size
y = zeros(length(t),size(y0,1)); 
y(1,:) = y0';                                          % initial condition

for i=1:(length(t)-1)                              % calculation loop
    dydt = fn_handle( t(i,1), y(i,:)' );
    y(i+1,:) = ( y(i,:)' + dydt )';  
    
    if stop_integration
        break;
    end
end

end

