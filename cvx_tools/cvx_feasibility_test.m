clear all;
clc;

E1 = zeros(4,4);
E1(1,1) = 1;
E1(2,2) = 1;
f1 = [1; 2; 0; 0];

E2 = zeros(4,4);
E2(2,2) = 1;
E2(3,3) = 1;
E2(4,4) = 1;
f2 = [0; 1; 3; 4];

x1 = pinv(E1)*f1;

x2 = pinv(E2)*f2;


x_opt = pinv([E1;E2])*[f1;f2];


A = [E1;E2];
b = [x1;x2];
cvx_begin
    variable x(4)
    
    subject to
        A*x  == b;
cvx_end
