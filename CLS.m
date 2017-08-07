function x_star = CLS( E, f, A, b )
%CLS Summary of this function goes here
%   Detailed explanation goes here
[m,n] = size(A);
E_reg = 0.00001*eye(n);
f_reg = zeros(n,1);
E_big = [E;E_reg];
f_big = [f;f_reg];
xhat = ([E_big'*E_big, A'; A, zeros(m,m) ])\[(E_big'*f_big);b];

% xhat = ([E'*E, A'; A, zeros(m,m) ])\[(E'*f);b];

x_star = xhat(1:n,1);
end

