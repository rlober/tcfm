clear;
clc;

load('A_mats_01.mat');
load('b_vecs_01.mat');

x1 = A1 \ b1;
x2 = A2 \ b2;
x3 = A3 \ b3;
x4 = A4 \ b4;
x5 = A5 \ b5;
x6 = A6 \ b6;

C = [x1,x2,x3,x4,x5,x6];

[n,m] = size(C);

% Create and solve the model
cvx_begin
    variable A(n,n) symmetric
    variable b(n)
    maximize( det_rootn( A ) )
    subject to
        norms( A * C + b * ones( 1, m ), 2 ) <= 1;
cvx_end

% Plot the results
clf
noangles = 200;
angles   = linspace( 0, 2 * pi, noangles );
ellipse  = A \ [ cos(angles) - b(1) ; sin(angles) - b(2) ];
plot(C(1,:), C(2,:), 'bo', 'MarkerSize', 5)
hold on
plot(ellipse(1,:), ellipse(2,:), 'g-');
comp_center = -1*A\b;
plot(comp_center(1), comp_center(2), 'gx', 'MarkerSize', 10)

%%
n = 2;
px = [0 .5 2 3 1]*5;
py = [0 1 1.5 .5 -.5]*5;
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

% formulate and solve the problem
cvx_begin
    variable B(n,n) symmetric
    variable d(n)
    maximize( det_rootn( B ) )
    subject to
       for i = 1:m
           norm( B*G(i,:)', 2 ) + G(i,:)*d <= h(i);
       end
cvx_end

% make the plots
ellipse_inner  = B * [ cos(angles) ; sin(angles) ] + d * ones( 1, noangles );
plot(px,py)
plot( ellipse_inner(1,:), ellipse_inner(2,:), 'r--' );
plot(d(1),d(2), 'rx', 'MarkerSize', 10)
axis square
hold off
legend('objective optima', 'objective ellipse', 'compatibility center', 'inequality constraints', 'constraint ellipse', 'feasibility center');