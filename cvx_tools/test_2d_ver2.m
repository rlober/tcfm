clear;
clc;


%% Get args
load('A_mats_01.mat');
load('b_vecs_01.mat');

x1 = A1 \ b1;
x2 = A2 \ b2;
x3 = A3 \ b3;
x4 = A4 \ b4;
x5 = A5 \ b5;
x6 = A6 \ b6;

C = [x1,x2,x3,x4,x5,x6];

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


%% solve
[obj_el, con_el] = computeConstraintAndObjectiveEllipses(C,G,h);

%% make the plots
% Plot the results
clf

plot(C(1,:), C(2,:), 'bo', 'MarkerSize', 5)
hold on
plot(obj_el.ellipse(1,:), obj_el.ellipse(2,:), 'g-');
plot(obj_el.center(1), obj_el.center(2), 'gx', 'MarkerSize', 10)
plot(px,py)
plot( con_el.ellipse(1,:), con_el.ellipse(2,:), 'r--' );
plot(con_el.center(1), con_el.center(2), 'rx', 'MarkerSize', 10)
axis square
hold off
legend('objective optima', 'objective ellipse', 'compatibility center', 'inequality constraints', 'constraint ellipse', 'feasibility center');