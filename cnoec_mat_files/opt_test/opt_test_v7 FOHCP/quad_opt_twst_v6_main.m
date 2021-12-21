%% Model parameters
Jx=0.0058319;
Jy=0.0058319;
Jz=0.0111886;
I_drone = [ ...
            Jx, 0, 0;
            0, Jy, 0;
            0, 0, Jz;
          ] ; % (kg.m^2)

mass = 0.9272;%(kg)
g=9.81;
Tmax=45;%(N)
taurollmax=0;%(Nm)
taupitchmax=0.58;
tauyawmin=-0.58;
th = [mass;Jx;Jy;Jz;g];

%% FHOCP parameters - single shooting
Ts      =       0.5;                % seconds, input sampling period
Tend    =       10;                 % seconds, terminal time
Np      =       Tend/Ts;            % prediction horizon

%% Initialize optimization variables
x0      =       [0;80/3.6;0;        % initial state: Y(m),speed(m/s),psi(rad)
                100*ones(Np,1);     % Torque (Nm),
                zeros(Np,1)];       % Steering angle (rad)

%% Constraints
%{
 Bounds on input variables
C       =       [-eye(2*Np+3)
                eye(2*Np+3)];
d       =       [-5;-150/3.6;-pi/3;
                 -Tdmax*ones(Np,1);
                 -dmax*ones(Np,1);
                 -5;50/3.6;-pi/3;
                 Tdmax*ones(Np,1)/10;
                 dmin*ones(Np,1)];
             
% Number of nonlinear inequality constraints (track borders)
q       =       2*Np;
%}
%% Solution -  BFGS
% Initialize solver options
myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'BFGS';
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-8;
myoptions.ls_beta       =	0.5;
myoptions.ls_c          =	.1;
myoptions.ls_nitermax   =	1e2;
myoptions.nitermax      =	1e3;
myoptions.xsequence     =	'on';
%myoptions.outputfcn     =   @(x)Vehicle_traj(x,Ts,Np,th);
f = @(x)quad_cost_constr(x,Ts,Np,th);
% Run solver
 C = 0;
 d = 0;
 q = 0;
[xstar,fxstar,niter,exitflag,xsequence] = myfmincon(@(x)quad_cost_constr(x,Ts,Np,th),x0,[],[],C,d,0,q,myoptions);

%% Visualize results
%[z_sim] = Vehicle_traj(xstar,Ts,Np,th);
