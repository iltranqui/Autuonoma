% Constrained Numerical Optimization for Estimation and Control
% Laboratory session E
% Script to test the unconstrained optimization algorithm on the Rosenbrock
% function

clear all
close all
clc

%% Rosenbrock function parameters
a               =   1;
b               =   100;

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

%% Initialize optimization variables
x0             	=   [5;5];

%% Matlab fminunc
% Initialize solver options
options                         =   optimoptions('fminunc');
options.OptimalityTolerance     =   1e-12;
options.Display                 =   'iter';
[xstar]=fminunc(@(x)Rosen(x,a,b),x0,options);

% Results
[xstar [a;a^2]]

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
[xstar,fxstar,niter,exitflag,xsequence] = myfminunc(@(x)Rosen(x,a,b),x0,myoptions);
figure,contour(x1vals,x2vals,log10(fvals),20),grid on,hold on
plot(xsequence(1,:),xsequence(2,:),'*-k'),title('Exact Newton')

% Results
[xstar [a;a^2]]

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
