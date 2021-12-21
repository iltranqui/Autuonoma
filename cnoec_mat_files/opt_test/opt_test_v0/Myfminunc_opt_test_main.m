% Constrained Numerical Optimization for Estimation and Control
% Laboratory session E
% Script to test the unconstrained optimization algorithm on the Rosenbrock
% function

clear all
close all
clc

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
alfa = zeros(N_points); %optimal parameter to be selected for the optimal trajectory if all == 0 x_cen = x_in and y_cen = y_in
ind = 1;
x_in_kp1= x_in(ind+1);
x_in_k = x_in(ind);
x_out_kp1=x_out(ind+1);
x_out_k = x_out(ind);

y_in_kp1= y_in(ind+1);
y_in_k = y_in(ind);
y_out_kp1=y_out(ind+1);
y_out_k = y_out(ind);
%{
%% Compute values for plotting
Nplot           =   5e2;       
x1vals          =   linspace(-15,15,Nplot);
x2vals          =   linspace(-15,30,Nplot);
fvals           =   zeros(Nplot);
for ind1 = 1:Nplot
    for ind2=1:Nplot
        fvals(ind2,ind1)    =   Rosen([x1vals(ind1);x2vals(ind2)],a,b);
    end
end
%}
%% Initialize optimization variables
x0             	=   [alfa(ind);alfa(ind+1)];
%{
%% Matlab fminunc
% Initialize solver options
options                         =   optimoptions('fminunc');
options.OptimalityTolerance     =   1e-12;
options.Display                 =   'iter';
[xstar]=fminunc(@(x)Rosen(x,a,b),x0,options);

% Results
[xstar [a;a^2]]
%}
%{
%% Gauss-Newton
% Initialize solver options
myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'GN';
myoptions.GN_funF       =	@(x)Rosen_F(x,a,b);
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-12;
myoptions.xsequence     =	'on';

% Run solver
[xstar,fxstar,niter,exitflag,xsequence] = myfminunc(@(x)Rosen(x,a,b),x0,myoptions);

% Results
[xstar [a;a^2]]
figure,contour(x1vals,x2vals,log10(fvals),20),grid on,hold on
plot(xsequence(1,:),xsequence(2,:),'*-k'),title('Gauss-Newton')
%}
%{
 %% BFGS
% Initialize solver options
myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'BFGS';
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-12;
myoptions.ls_tkmax      =	1;          
myoptions.ls_beta       =	0.5;
myoptions.ls_c          =	1e-4;
myoptions.ls_nitermax   =	1e2;
myoptions.nitermax      =	1e3;
myoptions.xsequence     =	'on';

% Run solver
[xstar,fxstar,niter,exitflag,xsequence] = myfminunc(@(x)Rosen(x,a,b),x0,myoptions);

% Results
[xstar [a;a^2]]
figure,contour(x1vals,x2vals,log10(fvals),20),grid on,hold on
plot(xsequence(1,:),xsequence(2,:),'*-k'),title('BFGS')
%}
%% Exact Newton
% Initialize solver options
myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'Exact';
myoptions.ls_tkmax      =	1;          
myoptions.ls_beta       =	0.5;
myoptions.ls_c          =	1e-4;
myoptions.ls_nitermax   =	100;
myoptions.nitermax      =	1e3;
myoptions.xsequence     =	'on';
% Run solver

%[f,gradient,hessf] = @(x)opt(x,x_in_kp1,x_in_k,x_out_kp1,x_out_k,y_in_kp1,y_in_k,y_out_kp1,y_out_k)
%xk   =   [1;1];
%[fxk,gradient] =  mygradient(@(x)opt(x,x_in_kp1,x_in_k,x_out_kp1,x_out_k,y_in_kp1,y_in_k,y_out_kp1,y_out_k),xk,'FD',2^-26)
%[gradfxk,hessf] =  mygradient(mygradient(@(x)opt(x,x_in_kp1,x_in_k,x_out_kp1,x_out_k,y_in_kp1,y_in_k,y_out_kp1,y_out_k),xk,'FD',2^-26),xk,'FD',2^-26)
%f = @(x)(x_in_kp1+ x(2,1)*(x_out_kp1- x_in_kp1)-(x_in_k+ x(1,1)*(x_out_k-x_in_k)))^2 ...
   % +(y_in_kp1+ x(2,1)*(y_in_kp1-y_out_kp1)-(y_in_k+ x(1,1)*(y_out_k-y_in_k)))^2
[xstar,fxstar,niter,exitflag,xsequence] = myfminunc(@(x)opt(x,x_in_kp1,x_in_k,x_out_kp1,x_out_k,y_in_kp1,y_in_k,y_out_kp1,y_out_k),x0,myoptions);
%[xstar,fxstar,niter,exitflag,xsequence] = myfminunc(@(x,y)opt(x,y,x_in_kp1,x_in_k,x_out_kp1,x_out_k,y_in_kp1,y_in_k,y_out_kp1,y_out_k),x0,myoptions);
%figure,contour(x1vals,x2vals,log10(fvals),20),grid on,hold on
%plot(xsequence(1,:),xsequence(2,:),'*-k'),title('Exact Newton')

% Results
%[xstar]

%alfa(ind) = xstar(1)
%alfa(ind + 1) = xstar(2);


%{
%% Steepest Descent
% Initialize solver options
myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'SD';
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-10;
myoptions.ls_tkmax      =	1;          
myoptions.ls_beta       =	0.1;
myoptions.ls_c          =	0.1;
myoptions.ls_nitermax   =	100;
myoptions.nitermax      =	1e3;
myoptions.xsequence     =	'on';
% Run solver
[xstar,fxstar,niter,exitflag,xsequence] = myfminunc(@(x)Rosen(x,a,b),x0,myoptions);

% Results
[xstar [a;a^2]]
figure,contour(x1vals,x2vals,log10(fvals),20),grid on,hold on
plot(xsequence(1,:),xsequence(2,:),'*-r'),title('Steepest descent')
%}