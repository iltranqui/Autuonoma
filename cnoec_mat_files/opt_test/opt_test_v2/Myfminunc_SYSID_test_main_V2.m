% Constrained Numerical Optimization for Estimation and Control
% Laboratory session E
% Script to test the unconstrained optimization algorithm on the nonlinear
% vehicle identification problem using simulation error criterion

clear all
close all
clc

% Model parameters
data_track = load('LVMS_ORC_NV.mat'); %Las Vegas Motor Speedway - Outside Road Course - North Variant 
kp = 1;
kl = 1;
th_opt_trajectory = [kp kl]; %kp,kl 
%% parameters
%inside points
x_in = data_track.Inside(:,1);
y_in= data_track.Inside(:,2);

%outside points
x_out= data_track.Outside(:,1);
y_out= data_track.Outside(:,2);

% initialization of coefficients alfa to be optimazed
N_points = length(x_in);

x0 = zeros(N_points,1); %optimal parameter to be selected for the optimal trajectory if all == 0 x_cen = x_in and y_cen = y_in

%{
ind = 1;
x_in_kp1= x_in(ind+1);
x_in_k = x_in(ind);
x_out_kp1=x_out(ind+1);
x_out_k = x_out(ind);

y_in_kp1= y_in(ind+1);
y_in_k = y_in(ind);
y_out_kp1=y_out(ind+1);
y_out_k = y_out(ind);
th      =       [x_in_kp1;x_out_kp1;x_in_k;x_out_k;y_in_kp1;y_out_kp1;y_in_k;y_out_k];
%}
%% Load data

%% Select weight matrix for cost function and scaling factors
Q               =   eye(3);
scaling         =   [1e-3,1e-4,1e-4,1e1,1]';                                % Scaling parameters for better conditioning
%% Initialize parameter estimate
                     


%% BFGS
% Initialize solver options
myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'BFGS';
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
%myoptions.graddx     =2.2204e-16;
myoptions.tolgrad    	=	1e-9;
myoptions.ls_tkmax      =	1;          
myoptions.ls_beta       =	0.5;
myoptions.ls_c          =	1e-4;
myoptions.ls_nitermax   =	100;
% Run solver

%{
for ind = 1 : N_points-1 
x_in_kp1= x_in(ind+1);
x_in_k = x_in(ind);
x_out_kp1=x_out(ind+1);
x_out_k = x_out(ind);

y_in_kp1= y_in(ind+1);
y_in_k = y_in(ind);
y_out_kp1=y_out(ind+1);
y_out_k = y_out(ind);
th      =       [x_in_kp1;x_out_kp1;x_in_k;x_out_k;y_in_kp1;y_out_kp1;y_in_k;y_out_k];
%}
[xstar,fxstar,niter,exitflag] = myfminunc(@(x)cost_lenght(x,x_in,x_out,y_in,y_out),x0,myoptions);


% Results
[xstar]
%}
%[~,ysim]               	=   vehicle_sim_cost(xstar,z0,uin,ymeas,th,Ts,Q,scaling);
%figure,plot(Data_1ms.time(1:downsampling:end),ymeas(1,:),Data_1ms.time(1:downsampling:end),ysim(1,:))
%figure,plot(Data_1ms.time(1:downsampling:end),ymeas(2,:),Data_1ms.time(1:downsampling:end),ysim(2,:))
%figure,plot(Data_1ms.time(1:downsampling:end),ymeas(3,:),Data_1ms.time(1:downsampling:end),ysim(3,:))
%{
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
for ind = 1 : N_points-1 
x_in_kp1= x_in(ind+1);
x_in_k = x_in(ind);
x_out_kp1=x_out(ind+1);
x_out_k = x_out(ind);

y_in_kp1= y_in(ind+1);
y_in_k = y_in(ind);
y_out_kp1=y_out(ind+1);
y_out_k = y_out(ind);
th      =       [x_in_kp1;x_out_kp1;x_in_k;x_out_k;y_in_kp1;y_out_kp1;y_in_k;y_out_k];

[xstar,fxstar,niter,exitflag] = myfminunc(@(x)cost_lenght(x,th,ind),x0,myoptions);
end
% Results
[xstar]
%}