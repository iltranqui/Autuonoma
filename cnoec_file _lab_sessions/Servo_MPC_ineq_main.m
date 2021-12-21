% Constrained Numerical Optimization for Estimation and Control
% Laboratory session F
% Script to set up and simulate a linear-quadratic MPC strategy
% for the linear servo-mechanism example, with inequality constraints and
% terminal equality constraint

clear all
close all
clc

%% System model
% Model parameters
R       =   20;             % Electric resistance
Ktheta  =   1280;           % Torsional stiffness output shaft
Kt      =   10;             % Motor constant
tau_g   =   20;             % Gear ratio
J_i     =   0.5;            % Input shaft moment of inertia
J_o     =   25;             % Output shaft moment of inertia
beta_i  =   0.1;            % Input shaft friction coefficient
beta_o  =   25;             % Output shaft friction coefficient
Tbar    =   78.5;           % Maximum torsional moment
Vbar    =   220;            % Maximum input voltage

Act = [0 0 1 0;
        0 0 0 1;
        -Ktheta/(J_i*tau_g^2) Ktheta/(J_i*tau_g) -(beta_i+Kt^2/R)/J_i 0;
        Ktheta/(J_o*tau_g) -Ktheta/J_o 0 -beta_o/J_o];

Bct = [0;0;Kt/(J_i*R);0];

Cct = [0 1 0 0];

Dct = 0;

Ts  =   0.1;            % Sampling time                       

Model           =   c2d(ss(Act,Bct,Cct,Dct),Ts);  % Discrete-time model with zoh

% System matrices:
[A,B,C,D]       =   ssdata(Model);                  % Model matrices 

%% Signal dimensions
nz     =   size(A,1);
nu      =   size(B,2);
ny      =   size(C,1);

%% Prediction horizon and cost function weighting matrices
N       =   7;
Q       =   1e4;
R       =   1e-5;

%% Inequality constraints
% Input inequalities
Au      =   [eye(nu);-eye(nu)];
bu      =   Vbar*ones(2*nu,1);
nqu     =   size(Au,1);                 % Number of input inequality constraints per stage

% State inequalities
Az     =   [Ktheta/tau_g -Ktheta 0 0;-Ktheta/tau_g +Ktheta 0 0];
bz     =   Tbar*ones(2,1);
nqz    =   size(Az,1);

%% Reference output and initial state
z0     =   zeros(nz,1);
yref    =   10*ones(ny,1);

%% Build overall matrices and vectors for QP (note - quadprog solves: min 0.5*x'*H*x + f'*x   subject to:  A*x <= b, Aeq*x = beq)
[Lambda_y,Gamma_y,Lambda_z,Gamma_z]   =   Traj_matrices(N,A,B,C,D);
Qbar                                    =   zeros((N+1)*ny);
Rbar                                    =   zeros(N*nu);
Yref                                    =   zeros((N+1)*ny,1);
Aubar                                   =   zeros(N*nqu,N*nu);
bubar                                   =   zeros(N*nqu,1);
Azbar                                  =   zeros((N+1)*nqz,(N+1)*nz);
bzbar                                  =   zeros((N+1)*nqz,1);

for ind = 1:N+1
    Qbar((ind-1)*ny+1:ind*ny,(ind-1)*ny+1:ind*ny)           =   Q;
    Yref((ind-1)*ny+1:ind*ny,1)                             =   yref;
    Azbar((ind-1)*nqz+1:ind*nqz,(ind-1)*nz+1:ind*nz)   =   Az;
    bzbar((ind-1)*nqz+1:ind*nqz,1)                       =   bz;
end

for ind = 1:N
    Rbar((ind-1)*nu+1:ind*nu,(ind-1)*nu+1:ind*nu)           =   R;
    Aubar((ind-1)*nqu+1:ind*nqu,(ind-1)*nu+1:ind*nu)        =   Au;
    bubar((ind-1)*nqu+1:ind*nqu,1)                          =   bu;
end

Aineq   =   [Aubar;Azbar*Gamma_z];
bineq   =   [bubar;bzbar-Azbar*Lambda_z*z0];

% Terminal equality constraint
Aeq     =   Gamma_z(end-nz+1:end,:)-Gamma_z(end-2*nz+1:end-nz,:);
beq     =   -(Lambda_z(end-nz+1:end,:)-Lambda_z(end-2*nz+1:end-nz,:))*z0;

% Cost function
f       =   z0'*Lambda_y'*Qbar*Gamma_y-Yref'*Qbar*Gamma_y;
H       =   (Gamma_y'*Qbar*Gamma_y)+Rbar;
H       =   0.5*(H+H');

%% QP options
options =   optimset('display','none');

%% Simulate with MPC
Nsim                =   150;
Zsim_MPC           =   zeros((Nsim+1)*nz,1);
Ysim_MPC            =   zeros(Nsim*ny,1);
Usim_MPC            =   zeros(Nsim*nu,1);
Zsim_MPC(1:nz,1)  =   z0;
zt                 =   z0;
tQP                 =   zeros(Nsim-1,1);

for ind=2:Nsim+1
    bineq                               =   [bubar;bzbar-Azbar*Lambda_z*zt];
    beq                                 =   -(Lambda_z(end-nz+1:end,:)-Lambda_z(end-2*nz+1:end-nz,:))*zt;
    f                                   =   zt'*Lambda_y'*Qbar*Gamma_y-Yref'*Qbar*Gamma_y;
    tic
    U                                   =   quadprog(H,f,Aineq,bineq,Aeq,beq,[],[],[],options);
    tQP(ind-1,1)                        =   toc;
    Usim_MPC((ind-2)*nu+1:(ind-1)*nu,1) =   U(1:nu,1);
    Zsim_MPC((ind-1)*nz+1:ind*nz,1)  =   A*Zsim_MPC((ind-2)*nz+1:(ind-1)*nz,1)+B*Usim_MPC((ind-2)*nu+1:(ind-1)*nu,1);
    Ysim_MPC((ind-2)*ny+1:(ind-1)*ny,1) =   C*Zsim_MPC((ind-2)*nz+1:(ind-1)*nz,1)+D*Usim_MPC((ind-2)*nu+1:(ind-1)*nu,1);
    zt                                 =   Zsim_MPC((ind-1)*nz+1:ind*nz,1);
end
figure(1),subplot(3,1,1),plot(Ts*[0:1:Nsim-1],Zsim_MPC(2:nz:Nsim*nz)),grid on, hold on,title('Output position')
xlabel('Time (s)')
subplot(3,1,2),plot(Ts*[0:1:(Nsim-1)]',Usim_MPC),grid on, hold on,title('Input voltage')
xlabel('Time (s)')
subplot(3,1,3),plot(Ts*[0:1:Nsim]',Ktheta*(Zsim_MPC(1:nz:end)/tau_g-Zsim_MPC(2:nz:end)))
grid on, hold on,title('Torsional stiffness')
xlabel('Time (s)')
figure(2),plot(Ts*[0:1:(Nsim-1)]',tQP),grid on, hold on,title('QP solution time (s)')

%% Compare with MPC without terminal equality constraint
Zsim_MPC_noeq              =   zeros((Nsim+1)*nz,1);
Ysim_MPC_noeq               =   zeros(Nsim*ny,1);
Usim_MPC_noeq               =   zeros(Nsim*nu,1);
Zsim_MPC_noeq(1:nz,1)     =   z0;
zt                         =   z0;

for ind=2:Nsim+1
    bineq                                       =   [bubar;bzbar-Azbar*Lambda_z*zt];
    f                                           =   zt'*Lambda_y'*Qbar*Gamma_y-Yref'*Qbar*Gamma_y;
    tic
    U                                           =   quadprog(H,f,Aineq,bineq,[],[],[],[],[],options);
    Usim_MPC_noeq((ind-2)*nu+1:(ind-1)*nu,1)    =   U(1:nu,1);
    Zsim_MPC_noeq((ind-1)*nz+1:ind*nz,1)     =   A*Zsim_MPC_noeq((ind-2)*nz+1:(ind-1)*nz,1)+B*Usim_MPC_noeq((ind-2)*nu+1:(ind-1)*nu,1);
    Ysim_MPC_noeq((ind-2)*ny+1:(ind-1)*ny,1)    =   C*Zsim_MPC_noeq((ind-2)*nz+1:(ind-1)*nz,1)+D*Usim_MPC_noeq((ind-2)*nu+1:(ind-1)*nu,1);
    zt                                         =   Zsim_MPC_noeq((ind-1)*nz+1:ind*nz,1);
end
figure(1),subplot(3,1,1),plot(Ts*[0:1:Nsim-1],Zsim_MPC_noeq(2:nz:Nsim*nz))
legend('MPC with terminal constraint','MPC without terminal constraint')
subplot(3,1,2),plot(Ts*[0:1:Nsim-1]',Usim_MPC_noeq)
legend('MPC with terminal constraint','MPC without terminal constraint')
subplot(3,1,3),plot(Ts*[0:1:Nsim]',Ktheta*(Zsim_MPC_noeq(1:nz:end)/tau_g-Zsim_MPC_noeq(2:nz:end)))
legend('MPC with terminal constraint','MPC without terminal constraint')
