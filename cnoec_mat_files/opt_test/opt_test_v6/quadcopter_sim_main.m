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
%% Simulation: Initial state
Vb_init   = [0;0;0]; % (m/sec) initial velocity in BODY axes
wb_init   = [0;0;0]; % (rad/sec) initial body rates
eul_init  = [0;0;0]; % (rad) initial euler angles (yaw,pitch,roll)
Xe_init   = [0;0;0]; % (m) initial position in inertial axes

%state vector INITIAL conditions
q_init = [ Xe_init;
           Vb_init;
           eul_init;
           wb_init];
       
%% Simulation: step amplitude
T_step             =    Tmax/2.5;
tauroll_step       =    taurollmax/2.5;
taupitch_step      =    taupitchmax/2.5;
tauyaw_step        =    tauyawmin/2.5;

%% Simulation with forward finite differences
% Time integration parameters
Ts_FFD      =       1e-1;               % sampling time (s)
Tend_FFD    =       200;                % final time (s)
tvec_FFD    =       0:Ts_FFD:Tend_FFD;  % time vector (s)

% Initialize simulation output
N_FFD               =       length(tvec_FFD);   % number of samples
zout_FFD            =       zeros(12,N_FFD);     % matrix with states
uout_FFD            =       zeros(4,N_FFD);     % matrix with inputs
zout_FFD(:,1)       =       q_init;
uout_FFD(:,1)       =       [T_step;tauroll_step;taupitch_step;tauyaw_step];

% Run simulation
for ind=2:N_FFD
    [zdot]              =   quadcopter(0,zout_FFD(:,ind-1),uout_FFD(:,ind-1),th);
    zout_FFD(:,ind)     =   zout_FFD(:,ind-1)+Ts_FFD*zdot;
    uout_FFD(:,ind)     =   uout_FFD(:,1);
end

% Plot the results
figure(1),plot(tvec_FFD,zout_FFD(1,:)),grid on, hold on,xlabel('Time (s)'),ylabel('X (m)')
figure(2),plot(tvec_FFD,zout_FFD(2,:)),grid on, hold on,xlabel('Time (s)'),ylabel('Y (m)')
figure(3),plot(tvec_FFD,zout_FFD(3,:)),grid on, hold on,xlabel('Time (s)'),ylabel('Z (m)')
figure(4),plot(tvec_FFD,zout_FFD(4,:)),grid on, hold on,xlabel('Time (s)'),ylabel('Body x speed (m/s)')
figure(5),plot(tvec_FFD,zout_FFD(5,:)),grid on, hold on,xlabel('Time (s)'),ylabel('Body y speed (m/s)')
figure(6),plot(tvec_FFD,zout_FFD(6,:)),grid on, hold on,xlabel('Time (s)'),ylabel('Body vertical speed (m/s)')
figure(7),plot(tvec_FFD,zout_FFD(7,:)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('yaw angle (deg)')
figure(8),plot(tvec_FFD,zout_FFD(8,:)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('pitch angle (deg)')
figure(9),plot(tvec_FFD,zout_FFD(9,:)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('roll angle (deg)')
figure(10),plot(tvec_FFD,zout_FFD(10,:)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('p rate (deg/s)')
figure(11),plot(tvec_FFD,zout_FFD(11,:)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('q rate (deg/s)')
figure(12),plot(tvec_FFD,zout_FFD(12,:)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('r rate (deg/s)')


%% Simulation with ode45
% Time integration parameters
Ts_o45      =       1e-1;               % sampling time (s)
Tend_o45    =       200;                % final time (s)
tvec_o45    =       0:Ts_o45:Tend_o45;  % time vector (s)

% Initialize simulation output
N_o45               =       length(tvec_o45);   % number of samples
zout_o45            =       zeros(12,N_o45);     % matrix with states
uout_o45            =       zeros(4,N_o45);     % matrix with inputs
zout_o45(:,1)       =       q_init;
uout_o45(:,1)       =       [1;1;1;1];

% Run simulation
for ind=2:N_o45
    zout_temp           =   ode45(@(t,z)quadcopter(t,z,uout_o45(:,ind-1),th),[0 Ts_o45],zout_o45(:,ind-1));
    zout_o45(:,ind)     =   zout_temp.y(:,end);
    uout_o45(:,ind)     =   uout_o45(:,1);
end

% Plot the results
figure(1),plot(tvec_o45,zout_o45(1,:)),grid on, hold on,xlabel('Time (s)'),ylabel('X (m)')
figure(2),plot(tvec_o45,zout_o45(2,:)),grid on, hold on,xlabel('Time (s)'),ylabel('Y (m)')
figure(3),plot(tvec_o45,zout_o45(3,:)),grid on, hold on,xlabel('Time (s)'),ylabel('Z (m)')
figure(4),plot(tvec_o45,zout_o45(4,:)),grid on, hold on,xlabel('Time (s)'),ylabel('xbdot (m/s)')
figure(5),plot(tvec_o45,zout_o45(5,:)),grid on, hold on,xlabel('Time (s)'),ylabel('ybdot (m/s)')
figure(6),plot(tvec_o45,zout_o45(6,:)),grid on, hold on,xlabel('Time (s)'),ylabel('zbdot (m/s)')
figure(7),plot(tvec_o45,zout_o45(7,:)),grid on, hold on,xlabel('Time (s)'),ylabel('yaw angle (deg)')
figure(8),plot(tvec_o45,zout_o45(8,:)),grid on, hold on,xlabel('Time (s)'),ylabel('pitch angle (deg)')
figure(9),plot(tvec_o45,zout_o45(9,:)),grid on, hold on,xlabel('Time (s)'),ylabel('roll angle (deg)')
figure(10),plot(tvec_o45,zout_o45(10,:)),grid on, hold on,xlabel('Time (s)'),ylabel('p rate (deg)')
figure(11),plot(tvec_o45,zout_o45(11,:)),grid on, hold on,xlabel('Time (s)'),ylabel('q rate (deg)')
figure(12),plot(tvec_o45,zout_o45(12,:)),grid on, hold on,xlabel('Time (s)'),ylabel('r rate angle (deg)')