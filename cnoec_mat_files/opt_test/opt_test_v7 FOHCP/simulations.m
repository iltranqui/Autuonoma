% Constrained Numerical Optimization for Estimation and Control
% Laboratory session A
% Script to initialize and simulate a 6 d.o.f. nonlinear vehicle model
% using different simulation approaches.
% Vehicle parameters taken from Canale et al., "Robust vehicle yaw control 
% using an active differential and IMC techniques", Control Engineering
% Practice 15, pp. 923-941, 2007

%% Initial commands
clear all
close all
clc

%% Model parameters
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

%% Simulation: Initial state
X       =       0;          % inertial X position (m)
Y       =       0;          % inertial Y position (m)
Ux      =       1;          % body x velocity (m/s)
beta    =       0;          % sideslip angle (rad)
psi     =       0;          % yaw angle (rad)
r       =       0;          % yaw rate (rad/s)
z0     =       [X;Y;Ux;beta;psi;r];

%% Simulation: step amplitude
Td_step             =       Tdmax/2.5;
delta_step          =       dmax/15;

%% Simulation with forward finite differences
% Time integration parameters
Ts_FFD      =       1e-1;               % sampling time (s)
Tend_FFD    =       200;                % final time (s)
tvec_FFD    =       0:Ts_FFD:Tend_FFD;  % time vector (s)

% Initialize simulation output
N_FFD               =       length(tvec_FFD);   % number of samples
zout_FFD            =       zeros(6,N_FFD);     % matrix with states
uout_FFD            =       zeros(2,N_FFD);     % matrix with inputs
Fout_FFD            =       zeros(6,N_FFD);     % matrix with forces
zout_FFD(:,1)       =       z0;
uout_FFD(:,1)       =       [Td_step;delta_step];

% Run simulation
for ind=2:N_FFD
    [zdot,F]            =   vehicle(0,zout_FFD(:,ind-1),uout_FFD(:,ind-1),0,th);
    zout_FFD(:,ind)     =   zout_FFD(:,ind-1)+Ts_FFD*zdot;
    Fout_FFD(:,ind)     =   F;
    uout_FFD(:,ind)     =   uout_FFD(:,1);
end

% Plot the results
figure(1),plot(zout_FFD(1,:),zout_FFD(2,:)),grid on, hold on,xlabel('X (m)'),ylabel('Y (m)')
figure(2),plot(tvec_FFD,zout_FFD(3,:)*3.6),grid on, hold on,xlabel('Time (s)'),ylabel('Longitudinal speed (km/h)')
figure(3),plot(tvec_FFD,zout_FFD(4,:)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('Sideslip angle (deg)')
figure(4),plot(tvec_FFD,zout_FFD(5,:)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('Yaw angle (deg)')
figure(5),plot(tvec_FFD,zout_FFD(6,:)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('Yaw rate (deg/s)')
figure(6),plot(tvec_FFD,Fout_FFD(1,:)),grid on, hold on,xlabel('Time (s)'),ylabel('Front lateral force (N)')
figure(7),plot(tvec_FFD,Fout_FFD(2,:)),grid on, hold on,xlabel('Time (s)'),ylabel('Rear lateral force (N)')
figure(8),plot(tvec_FFD,zout_FFD(6,:).*zout_FFD(3,:)/9.81),grid on, hold on,xlabel('Time (s)'),ylabel('Lateral acceleration (g)')
% figure(9),plot(zout_FFD(6,:).*zout_FFD(3,:),uout_FFD(2,:)*180/pi),grid on, hold on,xlabel('Lateral acceleration (m/s^2)'),ylabel('Steering angle (deg)')

%% Simulation with ode45
% Time integration parameters
Ts_o45      =       1e-1;               % sampling time (s)
Tend_o45    =       200;                % final time (s)
tvec_o45    =       0:Ts_o45:Tend_o45;  % time vector (s)

% Initialize simulation output
N_o45               =       length(tvec_o45);   % number of samples
zout_o45            =       zeros(6,N_o45);     % matrix with states
uout_o45            =       zeros(2,N_o45);     % matrix with inputs
Fout_o45            =       zeros(6,N_o45);     % matrix with forces
zout_o45(:,1)       =       z0;
uout_o45(:,1)       =       [Td_step;delta_step];

% Run simulation
for ind=2:N_o45
    zout_temp           =   ode45(@(t,z)vehicle(t,z,uout_o45(:,ind-1),0,th),[0 Ts_o45],zout_o45(:,ind-1));
    zout_o45(:,ind)     =   zout_temp.y(:,end);
    [~,F]               =   vehicle(0,zout_o45(:,ind),uout_o45(:,ind-1),0,th);
    Fout_o45(:,ind)     =   F;
    uout_o45(:,ind)     =   uout_o45(:,1);
end

% Plot the results
figure(1),plot(zout_o45(1,:),zout_o45(2,:)),grid on, hold on,xlabel('X (m)'),ylabel('Y (m)')
figure(2),plot(tvec_o45,zout_o45(3,:)*3.6),grid on, hold on,xlabel('Time (s)'),ylabel('Longitudinal speed (km/h)')
figure(3),plot(tvec_o45,zout_o45(4,:)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('Sideslip angle (deg)')
figure(4),plot(tvec_o45,zout_o45(5,:)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('Yaw angle (deg)')
figure(5),plot(tvec_o45,zout_o45(6,:)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('Yaw rate (deg/s)')
figure(6),plot(tvec_o45,Fout_o45(1,:)),grid on, hold on,xlabel('Time (s)'),ylabel('Front lateral force (N)')
figure(7),plot(tvec_o45,Fout_o45(2,:)),grid on, hold on,xlabel('Time (s)'),ylabel('Rear lateral force (N)')
figure(8),plot(tvec_o45,zout_o45(6,:).*zout_o45(3,:)/9.81),grid on, hold on,xlabel('Time (s)'),ylabel('Lateral acceleration (g)')
% figure(9),plot(zout_o45(6,:).*zout_o45(3,:),uout_o45(2,:)*180/pi),grid on, hold on,xlabel('Lateral acceleration (m/s^2)'),ylabel('Steering angle (deg)')

%% Simulation with Simulink
% Time integration parameters
Ts_slk      =       1e-1;               % sampling time (s)
Tend_slk    =       200;                % final time (s)

% Run simulation
sim('vehicle_sim',Tend_slk)

% Plot the results
figure(1),plot(zout_slk.signals.values(:,1),zout_slk.signals.values(:,2)),grid on, hold on,xlabel('X (m)'),ylabel('Y (m)')
figure(2),plot(zout_slk.time,zout_slk.signals.values(:,3)*3.6),grid on, hold on,xlabel('Time (s)'),ylabel('Longitudinal speed (km/h)')
figure(3),plot(zout_slk.time,zout_slk.signals.values(:,4)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('Sideslip angle (deg)')
figure(4),plot(zout_slk.time,zout_slk.signals.values(:,5)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('Yaw angle (deg)')
figure(5),plot(zout_slk.time,zout_slk.signals.values(:,6)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('Yaw rate (deg/s)')
figure(6),plot(zout_slk.time,Fout_slk.signals.values(:,1)),grid on, hold on,xlabel('Time (s)'),ylabel('Front lateral force (N)')
figure(7),plot(zout_slk.time,Fout_slk.signals.values(:,2)),grid on, hold on,xlabel('Time (s)'),ylabel('Rear lateral force (N)')
figure(8),plot(zout_slk.time,zout_slk.signals.values(:,6).*zout_slk.signals.values(:,3)/9.81),grid on, hold on,xlabel('Time (s)'),ylabel('Lateral acceleration (g)')
% figure(9),plot(zout_slk.signals.values(:,6).*zout_slk.signals.values(:,3),uout_slk.signals.values(:,2)*180/pi),grid on, hold on,xlabel('Lateral acceleration (m/s^2)'),ylabel('Steering angle (deg)')