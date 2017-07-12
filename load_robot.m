close all;
clear all; 
clc;

%% Load model
mdl_puma560;
global robot;
robot = p560;

robot.qlim(4,1) = -2.9671;

global qn;
% qn(1) =  3*pi/4;
qn(3) = -pi;
