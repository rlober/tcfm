clear all;
clc;


x1 = [0; -1];
x2 = [-3; 7];
x3 = [4; 4];
x4 = [2; 8];


a1 = 1;
E1 = zeros(2,2);
E1(2,2) = a1;
% E1 = eye(2);
f1 = x1*a1;

a2 = 1;
E2 = eye(2)*a2;
f2 = x2*a2;

a3 = 1;
E3 = eye(2)*a3;
f3 = x3*a3;

a4 = 1;
E4 = eye(2)*a4;
f4 = x4*a4;



x1 = pinv(E1) * f1;
x2 = pinv(E2) * f2;
x3 = pinv(E3) * f3;
x4 = pinv(E4) * f4;

C= [x1, x2, x3, x4];


g = 1;

Ereg = eye(2);
freg = zeros(2,1);
wreg = 0.00;

start = 0.0;
step = 10;
x_opts = zeros(2,step^4);

for i = start:1:step
    for j = start:1:step
        for k = start:1:step
            for m = start:1:step
                w1=i; w2=j; w3=k; w4=m;
                bigE = [sqrt(w1)*E1;sqrt(w2)*E2;sqrt(w3)*E3;sqrt(w4)*E4;sqrt(wreg)*Ereg];
                bigf = [sqrt(w1)*f1;sqrt(w2)*f2;sqrt(w3)*f3;sqrt(w4)*f4;sqrt(wreg)*freg];
                
                x_opts(:,g) = pinv(bigE)*bigf;
                g = g + 1;
            end
        end
    end
end


%%
cv_hull = [x1, x2, x4, x3, x1];
clf
plot(C(1,:), C(2,:), 'bo', 'MarkerSize', 10)
plot(cv_hull(1,:), cv_hull(2,:), 'b-', 'LineWidth', 2)
hold on
plot(x_opts(1,:), x_opts(2,:), 'go', 'MarkerSize', 5)

