clear all; 
clc;


    

%% Load model
mdl_puma560;
robot = p560;

q = qn;
tau = zeros(robot.n,1);

controller = QpController(robot);

step = 0.01;
tspan = [0.0 : step : 1.0];
y0 = [q'; zeros(robot.n,1)];
disp('simulating')
[t, y] = ode15s(@(t,y) dynamics(t,y,robot, controller, false), tspan, y0);
q_traj = y(:,1:robot.n);
disp('plotting')
tcp_traj = robot.fkine(q_traj);
end_point = tcp_traj(:,:,end)
robot.plot(q_traj)
% nsteps = size(t,1);
% for i = 1:nsteps
%     qnew = y(i, 1:robot.n);
%     robot.animate(qnew);
% end
