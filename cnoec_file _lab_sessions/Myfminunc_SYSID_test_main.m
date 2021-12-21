% Constrained Numerical Optimization for Estimation and Control
% Laboratory session E
% Script to test the unconstrained optimization algorithm on the nonlinear
% vehicle identification problem using simulation error criterion

clear all
close all
clc

%% Test with parameter identification problem of 3 d.o.f. nonlinear vehicle model
% Model parameters
m       =       1715;               % vehicle mass (kg)
Jz      =       2700;               % vehicle moment of inertia (kg*m^2)
a       =       1.07;               % distance between center of gravity and front axle (m)
b       =       1.47;               % distance between center of gravity and rear axle (m)
Cf      =       95117;              % front axle cornering stiffness (N/rad)
Cr      =       97556;              % rear axle cornering stiffness (N/rad)
rw      =       0.303;              % wheel radius (m)
mu      =       1;                  % road friction coefficient
Tdmax   =       1715*1.7*0.303;     % maximum driving torque (N*m)
Tdmin   =       -1715*9.81*0.303;   % maximum braking torque (N*m)
dmax    =       35*pi/180;          % maximum steering angle (rad)
dmin    =       -35*pi/180;         % minimum steering angle (rad)
Af      =       1.9;                % vehicle front surface (m^2)
Al      =       3.2;                % vehicle lateral surface (m^2)
Cx      =       0.4;                % vehicle front aerodynamic drag coefficient
Rr      =       0.016*m*9.81/30;    % rolling resistance coefficient(N*s/m)
rho     =       1.2;                % air density (kg/m^3)
th      =       [m;Jz;a;b;Cf;Cr;rw;mu;Tdmax;Tdmin;dmax;dmin;Af;Al;Cx;Rr;rho];

%% Load data
% load vehicle_data_maneuver_4.mat
load vehicle_data_maneuver_1.mat
% load vehicle_data_maneuver_2.mat

downsampling 	=   10;                                                     % Reduce sampling frequency for higher simulation speed with FFD
Ts             	=   downsampling*Ts_1ms;                                    % 10 ms sampling time
uin             =   Vehicle_input.signals.values(1:downsampling:end,:)';    % Inputs
ymeas          	=   [Data_1ms.signals.values(1:downsampling:end,1)';        % Measured forward speed
                   	Data_1ms.signals.values(1:downsampling:end,2)';         % Measured side-slip angle
                   	Data_1ms.signals.values(1:downsampling:end,3)'];        % Measured yaw rate
z0            	=   [0;0;80/3.6;0;0;0];                                     % Initial condition

%% Select weight matrix for cost function and scaling factors
Q               =   eye(3);
scaling         =   [1e-3,1e-4,1e-4,1e1,1]';                                % Scaling parameters for better conditioning
%% Initialize parameter estimate
x0             	=   [2000;70000;65000;0.6;1].*scaling;                      % Jz, Cf, Cr, Cx, Rr

%% Gauss-Newton
% Initialize solver options
myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'GN';
myoptions.GN_funF       =	@(x)vehicle_sim_err(x,z0,uin,ymeas,th,Ts,Q,scaling);
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-9;

% Run solver
[xstar,fxstar,niter,exitflag] = myfminunc(@(x)vehicle_sim_cost(x,z0,uin,ymeas,th,Ts,Q,scaling),x0,myoptions);

% Results
[xstar./scaling [Jz;Cf;Cr;Cx;Rr]]
[~,ysim]               	=   vehicle_sim_cost(xstar,z0,uin,ymeas,th,Ts,Q,scaling);
figure,plot(Data_1ms.time(1:downsampling:end),ymeas(1,:),Data_1ms.time(1:downsampling:end),ysim(1,:))
figure,plot(Data_1ms.time(1:downsampling:end),ymeas(2,:),Data_1ms.time(1:downsampling:end),ysim(2,:))
figure,plot(Data_1ms.time(1:downsampling:end),ymeas(3,:),Data_1ms.time(1:downsampling:end),ysim(3,:))

%% BFGS
% Initialize solver options
myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'BFGS';
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-9;
myoptions.ls_tkmax      =	1;          
myoptions.ls_beta       =	0.5;
myoptions.ls_c          =	1e-4;
myoptions.ls_nitermax   =	100;
% Run solver
[xstar,fxstar,niter,exitflag] = myfminunc(@(x)vehicle_sim_cost(x,z0,uin,ymeas,th,Ts,Q,scaling),x0,myoptions);

% Results
[xstar./scaling [Jz;Cf;Cr;Cx;Rr]]
[~,ysim]               	=   vehicle_sim_cost(xstar,z0,uin,ymeas,th,Ts,Q,scaling);
figure,plot(Data_1ms.time(1:downsampling:end),ymeas(1,:),Data_1ms.time(1:downsampling:end),ysim(1,:))
figure,plot(Data_1ms.time(1:downsampling:end),ymeas(2,:),Data_1ms.time(1:downsampling:end),ysim(2,:))
figure,plot(Data_1ms.time(1:downsampling:end),ymeas(3,:),Data_1ms.time(1:downsampling:end),ysim(3,:))

%% Steepest Descent
% Initialize solver options
myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'SD';
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-8;
myoptions.ls_tkmax      =	2;          
myoptions.ls_beta       =	0.8;
myoptions.ls_c          =	0.1;
myoptions.ls_nitermax   =	40;
myoptions.nitermax      =	1e3;
% Run solver
[xstar,fxstar,niter,exitflag] = myfminunc(@(x)vehicle_sim_cost(x,z0,uin,ymeas,th,Ts,Q,scaling),x0,myoptions);

% Results
[xstar./scaling [Jz;Cf;Cr;Cx;Rr]]
[~,ysim]               	=   vehicle_sim_cost(xstar,z0,uin,ymeas,th,Ts,Q,scaling);
figure,plot(Data_1ms.time(1:downsampling:end),ymeas(1,:),Data_1ms.time(1:downsampling:end),ysim(1,:))
figure,plot(Data_1ms.time(1:downsampling:end),ymeas(2,:),Data_1ms.time(1:downsampling:end),ysim(2,:))
figure,plot(Data_1ms.time(1:downsampling:end),ymeas(3,:),Data_1ms.time(1:downsampling:end),ysim(3,:))