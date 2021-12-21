% Constrained Numerical Optimization for Estimation and Control
% Laboratory session F
% Script to formulate and solve a Finite Horizon Optimal Control Problem
% for the linear servo-mechanism example, with terminal equality constraint
% only

clear all
close all
clc

%% System model
% Model parameters
R       = 20;           % Electric resistance
Ktheta  = 1280;         % Torsional stiffness output shaft
Kt      = 10;           % Motor constant
tau_g   = 20;           % Gear ratio
J_i     = .5;           % Input shaft moment of inertia
J_o     = 25;           % Output shaft moment of inertia
beta_i  = 0.1;          % Input shaft friction coefficient
beta_o  = 25;           % Output shaft friction coefficient

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
N       =   50;                  % prediction horizon
Q       =   1e4;                % Output weights
R       =   1e-3;               % Input variation weights

%% Reference output and initial state
z0     =   0*ones(nz,1);
yref    =   10*ones(ny,1);

%% Build overall matrices and vectors for KKT system
[Lambda_y,Gamma_y,Lambda_z,Gamma_z]   =   Traj_matrices(N,A,B,C,D);
Qbar                                    =   zeros((N+1)*ny);
Rbar                                    =   zeros(N*nu);
Yref                                    =   zeros((N+1)*ny,1);

for ind = 1:N+1
    Qbar((ind-1)*ny+1:ind*ny,(ind-1)*ny+1:ind*ny)   =   Q;
    Yref((ind-1)*ny+1:ind*ny,1)                     =   yref;
end

for ind = 1:N
    Rbar((ind-1)*nu+1:ind*nu,(ind-1)*nu+1:ind*nu)   =   R;
end

P       =   [Gamma_y'*Qbar*Gamma_y+Rbar -(Gamma_z(end-nz+1:end,:)-Gamma_z(end-2*nz+1:end-nz,:))';
            Gamma_z(end-nz+1:end,:)-Gamma_z(end-2*nz+1:end-nz,:) zeros(nz)];
S       =   [-Gamma_y'*Qbar*Lambda_y Gamma_y'*Qbar;
            -(Lambda_z(end-nz+1:end,:)-Lambda_z(end-2*nz+1:end-nz,:)) zeros(nz,(N+1)*ny)];

Gains   =   P\S;

%% Compute optimal input sequence - terminal equality constraint - reference tracking
sol     =   Gains*[z0;Yref];
U       =   sol(1:nu*N,1);

%% Plot results
Z      =   Lambda_z*z0+Gamma_z*U;
figure(1),subplot(3,1,2),plot(Ts*[0:1:N-1]',U),grid on, hold on,title('Input voltage')
xlabel('Time (s)')
subplot(3,1,1),plot(Ts*[0:1:N]',Lambda_y*z0+Gamma_y*U),grid on, hold on,title('Output position')
xlabel('Time (s)')
subplot(3,1,3),plot(Ts*[0:1:N]',Ktheta*(Z(1:nz:end)/tau_g-Z(2:nz:end))),grid on, hold on,title('Torsional stiffness')
xlabel('Time (s)')

%% Check terminal equality constraint
Z      =   Lambda_z*z0+Gamma_z*U;
Z(end-2*nz+1:end-nz)-Z(end-nz+1:end);

%% Extract feedback gain
Keq     =   Gains(1:nu,1:nz);

%% Compute optimal input sequence - terminal equality constraint - regulation to the origin
z0     =   [tau_g*10;10;0;0];
yref    =   zeros(ny,1);
Yref    =   zeros((N+1)*ny,1);

for ind = 1:N+1
    Yref((ind-1)*ny+1:ind*ny,1) =   yref;
end

sol     =   Gains*[z0;Yref];
U       =   sol(1:nu*N,1);

%% Plot results
Z      =   Lambda_z*z0+Gamma_z*U;
figure(2),subplot(3,1,2),plot(Ts*[0:1:N-1]',U),grid on, hold on,title('Input voltage')
xlabel('Time (s)')
subplot(3,1,1),plot(Ts*[0:1:N]',Lambda_y*z0+Gamma_y*U),grid on, hold on,title('Output position')
xlabel('Time (s)')
subplot(3,1,3),plot(Ts*[0:1:N]',Ktheta*(Z(1:nz:end)/tau_g-Z(2:nz:end))),grid on, hold on,title('Torsional stiffness')
xlabel('Time (s)')

%% Simulate with state feedback (explicit feedback law)
Nsim                =   150;
Z_fb               =   zeros(Nsim+1*nz,1);
Y_fb                =   zeros(Nsim+1*ny,1);
U_fb                =   zeros(Nsim*nu,1);
Z_fb(1:nz,1)      =   z0;
Y_fb(1:ny,1)        =   (C+D*Keq)*z0;
U_fb(1:nu,1)        =   Keq*z0;
for ind = 2:Nsim+1
    Z_fb((ind-1)*nz+1:ind*nz,1)  =   (A+B*Keq)*Z_fb((ind-2)*nz+1:(ind-1)*nz,1);
    U_fb((ind-1)*nu+1:ind*nu,1)     =   Keq*Z_fb((ind-1)*nz+1:ind*nz,1);
    Y_fb((ind-1)*ny+1:ind*ny,1)     =   (C+D*Keq)*Z_fb((ind-1)*nz+1:ind*nz,1);
end
figure(3),subplot(3,1,2),plot(Ts*[0:1:Nsim]',U_fb),grid on, hold on,title('Input voltage')
xlabel('Time (s)')
subplot(3,1,1),plot(Ts*[0:1:Nsim]',Z_fb(2:nz:end)),grid on, hold on,title('Output position')
xlabel('Time (s)')
subplot(3,1,3),plot(Ts*[0:1:Nsim]',Ktheta*(Z_fb(1:nz:end)/tau_g-Z_fb(2:nz:end))),grid on, hold on,title('Torsional stiffness')
xlabel('Time (s)')

% Compare with LQR simulation
Qlqr                =   zeros(nz);
Qlqr(2,2)           =   Q;
Klqr                =   -dlqr(A,B,Qlqr,R);
Z_lqr              =   zeros(Nsim+1*nz,1);
Y_lqr               =   zeros(Nsim+1*ny,1);
U_lqr               =   zeros(Nsim*nu,1);
Z_lqr(1:nz,1)     =   z0;
Y_lqr(1:ny,1)       =   (C+D*Klqr)*z0;
U_lqr(1:nu,1)       =   Keq*z0;
for ind = 2:Nsim+1
    Z_lqr((ind-1)*nz+1:ind*nz,1) =   (A+B*Klqr)*Z_lqr((ind-2)*nz+1:(ind-1)*nz,1);
    U_lqr((ind-1)*nu+1:ind*nu,1)     =   Keq*Z_lqr((ind-1)*nz+1:ind*nz,1);
    Y_lqr((ind-1)*ny+1:ind*ny,1)    =   (C+D*Klqr)*Z_lqr((ind-1)*nz+1:ind*nz,1);
end
figure(3),subplot(3,1,2),plot(Ts*[0:1:Nsim]',U_lqr)
legend('Explicti QP solution','LQR')
subplot(3,1,1),plot(Ts*[0:1:Nsim]',Z_lqr(2:nz:end))
legend('Explicti QP solution','LQR')
subplot(3,1,3),plot(Ts*[0:1:Nsim]',Ktheta*(Z_lqr(1:nz:end)/tau_g-Z_lqr(2:nz:end)))
legend('Explicti QP solution','LQR')
