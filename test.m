clear all; 
clc;


    

%% Load model
mdl_puma560;
robot = p560;

q = qn;
ee = robot.fkine(q);
ee_pos = ee(1:3,4);

nsteps = 10;




x = 0.5;
y = 0.5;
z = 0.5;

xs = linspace(ee_pos(1,1), x, nsteps);
ys = linspace(ee_pos(2,1), y, nsteps);
zs = linspace(ee_pos(3,1), z, nsteps);

robot.plot(q)
for i = 1:nsteps
    ee = robot.fkine(q);
    Jinv = pinv(robot.jacob0(q));
    xdes = [1 0 0 xs(i); 0 1 0 ys(i); 0 0 1 zs(i); 0 0 0 1];
    dx = tr2delta(ee, xdes);
    qnew = Jinv * dx + q';
    robot.animate(qnew');
    q = qnew';
end


% robot.nofriction();
% robot.inertia(qz)

% robot.j
% [t,q] = robot.fdyn(0.002, @qp_control, qn, qz);
% 
% q1 = linspace(0.0, 3.14, 100);
% q = [q1',q1',q1',q1',q1',q1'];
% robot.plot(q(1,:))
% for i = 2:size(q)
%     robot.animate(q(i,:));
% end
