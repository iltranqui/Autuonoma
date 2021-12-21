%% cnoec project = quadrotor optimization cost function
%% what we have to do
% see slide chapter 3 acav2 "cnoec project"
% 
% N.B : da implementare con i vettori usando gli indici
% 
% recap : 
% 
% from the boundaries of the track, the algorithm define the optimal
% 
% trajectory to follow in order to decrease the lenght and the curvature
% 
% based on that trajectory the speed profile following the lateral and
% 
% longitudinal acceleration constraints are defined in order to get the
% 
% lower lap time.
% 
% functions : 
% 
% - 1 for the trajectory
% 
% - 2 for the speed profile and the definition of the lap time
% 
% - 3 for the track boundaries
% 
% - 4 for the other possible cost functions
% 
% -5 for the definition of the optimal parameters to be used in order to
% 
% obtain the optimal tajecotory, using the myfminfunc used during labs to
% 
% define the optimal parameters vector
% 
% - 6 one for simulation
% 
% - 7 for the test of the model
% 
% - 8 for the model 
% 
% - 9 extra:
% 
% * it is possible to define the lap time or also make a tracking algoritm
% 
% type considering the value of a possible lap time and set the parameters
% 
% to obtain that.
%% clear the workspace and the command window

clear all
close all 
clc


data_track = load('LVMS_ORC_NV.mat'); %Las Vegas Motor Speedway - Outside Road Course - North Variant 
kp = 1;
kl = 1;
th_opt_trajectory = [kp kl]; %kp,kl 
[opt_trajectory_data , opt_trajectory] = quadcopter_opt_trajectory(data_track,th_opt_trajectory);

%% model parameters
%% create the cost function to be minimized
% done as a matlab function
% 
% use myfminunc done by the professor to find the parameters that minimize the 
% cost function considered
%% test for the minimization
xk   =   [1;1];     % Selected point
format long
tic
[fxk,gradient_UP]	=	mygradient(@(x)test(x),xk,'UP',1e-8)   % Exact gradient
toc
tic
[~,gradient_FD]     =	mygradient(@(x)test(x),xk,'FD',2^-26)  % Forward Finite Differences
toc
tic
[~,gradient_CD]     =	mygradient(@(x)test(x),xk,'CD',2^-17)  % Central Finite Differences
toc
tic
[~,gradient_IM]     =	mygradient(@(x)test(x),xk,'IM',1e-8)   % Imaginary-part trick
toc
