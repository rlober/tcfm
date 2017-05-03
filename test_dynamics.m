clear all; 
clc;


    

%% Load model
mdl_puma560;
robot = p560;

q = qn;
tau = zeros(robot.n,1);

tspan = [0.0 1.0];
y0 = [q'; zeros(robot.n,1)];
disp('simulating')
[t, y] = ode45(@(t,y) dynamics(t,y,robot, @task, false), tspan, y0);

disp('plotting')
robot.plot(y(:,1:robot.n))
% nsteps = size(t,1);
% for i = 1:nsteps
%     qnew = y(i, 1:robot.n);
%     robot.animate(qnew);
% end
