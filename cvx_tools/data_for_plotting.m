clear all;
clc;

x1 = [0; -1];
x2 = [-3; 7];
x3 = [4; 4];
x4 = [2; 8];


a1 = 1;
E1 = zeros(2,2);
E1(2,2) = a1;
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

w1 = 0.002;
w2 = 0.8;
w3 = 0.5; 
w4 = 1.0;

wsum = w1 + w2 + w3 + w4;

w1 = w1/wsum;
w2 = w2/wsum;
w3 = w3/wsum;
w4 = w4/wsum;

w1=1; w2=1; w3=1; w4=1;




C = [x1,x2,x3,x4];

n = 2;
px = [0 .5 2 3 1]*4;
py = [0 1 1.5 .5 -.5]*4;
m = size(px,2);
pxint = sum(px)/m; pyint = sum(py)/m;
px = [px px(1)];
py = [py py(1)];

% generate G,h
G = zeros(m,n); h = zeros(m,1);
for i=1:m
  G(i,:) = null([px(i+1)-px(i) py(i+1)-py(i)])';
  h(i) = G(i,:)*.5*[px(i+1)+px(i); py(i+1)+py(i)];
  if G(i,:)*[pxint; pyint]-h(i)>0
    G(i,:) = -G(i,:);
    h(i) = -h(i);
  end
end


E_array = {E1, E2, E3, E4};
f_array = {f1, f2, f3, f4};
bigE = [sqrt(w1)*E1;sqrt(w2)*E2;sqrt(w3)*E3;sqrt(w4)*E4];
bigf = [sqrt(w1)*f1;sqrt(w2)*f2;sqrt(w3)*f3;sqrt(w4)*f4];
% bigE = [w1*E1;w2*E2;w3*E3;w4*E4];
% bigf = [w1*f1;w2*f2;w3*f3;w4*f4];
% bigE = E4;
% bigf = f4;
x_opt = pinv(bigE)*bigf;

Q = (bigE')*bigE;
p = -(bigE'*bigf)';
options = optimset('Algorithm','interior-point-convex');

x_opt_weighted = quadprog(Q, p, G, h, [],[],[],[],[],options);
x_opt_weighted_unconstrained = quadprog(Q, p, [], [], [],[],[],[],[],options);

x_opt_hierarchical = hierarchicalQP(E_array, f_array, G, h);
x_opt_hierarchical_unconstrained = hierarchicalQP(E_array, f_array, [], []);


k_w_1 = norm((E1*x_opt_weighted_unconstrained - f1),2)^2;
k_w_2 = norm((E2*x_opt_weighted_unconstrained - f2),2)^2;
k_w_3 = norm((E3*x_opt_weighted_unconstrained - f3),2)^2;
k_w_4 = norm((E4*x_opt_weighted_unconstrained - f4),2)^2;

K_w = [k_w_1, k_w_2, k_w_3, k_w_4];

k_h_1 = norm((E1*x_opt_hierarchical_unconstrained - f1),2)^2;
k_h_2 = norm((E2*x_opt_hierarchical_unconstrained - f2),2)^2;
k_h_3 = norm((E3*x_opt_hierarchical_unconstrained - f3),2)^2;
k_h_4 = norm((E4*x_opt_hierarchical_unconstrained - f4),2)^2;

K_h = [k_h_1, k_h_2, k_h_3, k_h_4];
% cvx_begin
%     variable x_opt_constrained(2)
%     minimize( power(2, norm(bigE*x_opt_constrained - bigf,2)) )
%     subject to
%         G*x_opt_constrained <= h;
% cvx_end

%% solve
[obj_el, con_el] = computeConstraintAndObjectiveEllipses(C,G,h);
Es = {E1, E2, E3, E4};
fs = {f1, f2, f3, f4};
weights = [1, 1, 1, 1];
[ compatibility_metrics, feasibility_metrics ] = computeMetrics(weights, Es, fs, C, x_opt_weighted_unconstrained, obj_el, con_el);
%%
% [v_obj, lambda_obj] = eig(obj_el.A)
% [v_con, lambda_con] = eig(con_el.B)

[U,S_obj,V] = svd(obj_el.B)
us_obj = U*S_obj
obj_el.volume = det(S_obj)

obj_el.center + us_obj(:,1)

[U,S_con,V] = svd(con_el.B)
us_con = U*S_con
con_el.volume = det(S_con)

for i = 1:size(C,2)
    x = C(:,i);
    el_eval = (x - con_el.d)' * con_el.B^(-2) * (x - con_el.d);
    if el_eval <= 1.0
        fprintf('The point x%i is in the constraint ellipsoid', i)
    end
end


% Note that because of the way the ellipse is parameterized, the eigen
% values of con_el.B are equal to the length of the radii. Doing SVD on B
% gives the same results as eye(B)... These two things below are
% equivalent.
% sqrt(eig(con_el.B^2))
% [a, b] = eig(con_el.B)

%%
for i = 1:4
    disp(i)
    err = C(:,i) - x_opt_weighted_unconstrained;
    norm(err,1)
    norm(err,2)
    norm(err,'fro')
    norm(err,Inf)

end



%% make the plots
% Plot the results
clf

plot(C(1,:), C(2,:), 'bo', 'MarkerSize', 5)
hold on
plot(x_opt(1,:), x_opt(2,:), 'bo', 'MarkerSize', 10)
plot(x_opt_weighted(1,:), x_opt_weighted(2,:), 'ko', 'MarkerSize', 10)
plot(x_opt_hierarchical(1,:), x_opt_hierarchical(2,:), 'kx', 'MarkerSize', 10)
plot(linspace(-4,14,100),ones(100)*x1(2), 'y-')
plot(obj_el.ellipse(1,:), obj_el.ellipse(2,:), 'g-');

obj_radii = {};
for i = 1:size(us_obj,2)
    point = obj_el.center + us_obj(:,i);
    radii = [obj_el.center, point];
    obj_radii = [obj_radii, radii];
    plot(radii(1,:), radii(2,:), 'g--')
end



plot(obj_el.center(1), obj_el.center(2), 'gx', 'MarkerSize', 10)
plot(px,py)
plot( con_el.ellipse(1,:), con_el.ellipse(2,:), 'r--' );
con_radii = {};
for i = 1:size(us_con,2)
    point = con_el.center - us_con(:,i);
    radii = [con_el.center, point];
    con_radii = [con_radii, radii];
    plot(radii(1,:), radii(2,:), 'r--')
end
plot(con_el.center(1), con_el.center(2), 'rx', 'MarkerSize', 10)
xlim([-4,14])
ylim([-6,12])
axis square
hold off
% legend('objective optima', 'objective ellipse', 'compatibility center', 'inequality constraints', 'constraint ellipse', 'feasibility center');


save('../example_data');