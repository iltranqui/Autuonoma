function    [err_vec,ysim]    =   vehicle_sim_err(x,z0,uin,ymeas,th,Ts,Q,scaling)
% Function that computes the output trajectory of the vehicle model
% introduced in Lab. session A and the weighted error between the simulated output
% and the measured one. Variable x corresponds to the vehicle parameters 
% to be estimated, possibly scaled. The simulation is carried out with forward finite
% differences

%% Model parameters
x       =       x./scaling;
m       =       th(1,1);            % vehicle mass (kg)
Jz      =       x(1,1);             % vehicle moment of inertia (kg*m^2)
a       =       th(3,1);            % distance between center of gravity and front axle (m)
b       =       th(4,1);            % distance between center of gravity and rear axle (m)
Cf      =       x(2,1);             % front axle cornering stiffness (N/rad)
Cr      =       x(3,1);             % rear axle cornering stiffness (N/rad)
rw      =       th(7,1);            % wheel radius (m)
mu      =       th(8,1);            % road friction coefficient
Tdmax   =       th(9,1);            % maximum driving torque (N*m)
Tdmin   =       th(10,1);           % maximum braking torque (N*m)
dmax    =       th(11,1);           % maximum steering angle (rad)
dmin    =       th(12,1);           % minimum steering angle (rad)
Af      =       th(13,1);           % vehicle front surface (m^2)
Al      =       th(14,1);           % vehicle lateral surface (m^2)
Cx      =       x(4,1);             % vehicle front aerodynamic drag coefficient
Rr      =       x(5,1);             % rolling resistance coefficient(N*s/m)
rho     =       th(17,1);           % air density (kg/m^3)
th      =       [m;Jz;a;b;Cf;Cr;rw;mu;Tdmax;Tdmin;dmax;dmin;Af;Al;Cx;Rr;rho];

%% Initialize simulation output
N_FFD               =       size(ymeas,2);      % number of samples
zhat_FFD            =       zeros(6,N_FFD);     % matrix with states
zhat_FFD(:,1)       =       z0;

%% Run simulation with FFD
for ind=2:N_FFD
    zdot               =   vehicle(0,zhat_FFD(:,ind-1),uin(:,ind-1),0,th);
    zhat_FFD(:,ind)    =   zhat_FFD(:,ind-1)+Ts*zdot;
end

%% Collect simulated output (forward speed and yaw rate)
ysim    =   [zhat_FFD(3,:);zhat_FFD(4,:);zhat_FFD(6,:)]; % forward speed, side-slip, yaw rate

%% Compute weighted errors
err     =   Q*(ymeas-ysim);

%% Stack errors in one vector
err_vec =   [err(1,:) err(2,:) err(3,:)]'/sqrt(N_FFD);
