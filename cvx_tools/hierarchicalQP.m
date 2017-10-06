function [ x, fval, status ] = hierarchicalQP( E_array, f_array, G, h )
%HIERARCHICALQP Summary of this function goes here
%   Detailed explanation goes here

n_objectives = size(E_array,2);
options = optimset('Algorithm','interior-point-convex');
A = [];
b = [];
for i = 1:n_objectives 
    disp(i)
    E = [E_array{i}; eye(size(E_array{i},1))*0.1];
    f = [f_array{i}; zeros(2,1)];
    Q = (E')*E;
    p = -(E'*f)';

    [ x_tmp, fval_tmp, status_tmp ] = quadprog(Q, p, G, h, A, b,[],[],[],options);

    if status_tmp == 0 || status_tmp == 1
        A = [A; E(1:size(E_array{i},1), 1:size(E_array{i},2))];
        b = [b; x_tmp];
        x = x_tmp; fval = fval_tmp; status = status_tmp;
    else
        break;
    end
end

