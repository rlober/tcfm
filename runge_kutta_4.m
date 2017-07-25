function [ t, y ] = runge_kutta_4( fn_handle, tspan, y0 )
global stop_integration;
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
t = tspan';
h=t(2,1)-t(1,1);                                             % step size
y = zeros(length(t),size(y0,1)); 
y(1,:) = y0';                                          % initial condition

for i=1:(length(t)-1)                              % calculation loop
    k_1 = fn_handle(   t(i,1)          ,   y(i,:)'               );
    k_2 = fn_handle( ( t(i,1) + 0.5*h ), ( y(i,:)' + 0.5*h*k_1 ) );
    k_3 = fn_handle( ( t(i,1) + 0.5*h ), ( y(i,:)' + 0.5*h*k_2 ) );
    k_4 = fn_handle( ( t(i,1) + h     ), ( y(i,:)' + k_3*h )     );

    y(i+1,:) = ( y(i,:)' + (1/6) * ( k_1 + 2*k_2 + 2*k_3 + k_4 )*h )';  
    
    if stop_integration
        break;
    end
end

end

