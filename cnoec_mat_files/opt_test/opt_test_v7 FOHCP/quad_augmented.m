function [zdot]=quad_augmented(t,z,u,d,th)
%% Read parameters, states and inputs
%{
% u= agumented inputs
  d = tau,roll,yaw torques
%}
% Parameters
mass = th(1,1);
Jx = th(2,1);
Jy = th(3,1);
Jz = th(4,1);
g = th(5,1);

% States
X        =       z(1,1);    % inertial X position (m)
Y        =       z(2,1);    % inertial Y position (m)
Z        =       z(3,1);    % inertial Z position (m)

xbdot     =       z(4,1);    % body x velocity (m/s)
ybdot     =       z(5,1);    % body y velocity (m/s)
zbdot     =       z(6,1);    % body z velocity (m/s)

psi      =       z(7,1);    % yaw angle   (rad)
theta    =       z(8,1);    % pitch angle (rad)
phi      =       z(9,1);    % roll angle  (rad)

p        =       z(10,1);   % inertial p rate (rad/s)
q        =       z(11,1);   % inertial q rate (rad/s)
r        =       z(12,1);   % inertial r rate (rad/s)

% additional states
progress =       z(13,1); % to define the position of the quadcopter w.r.t the trajectory
progress_speed = z(14,1); 
T = z(15,1);

% Inputs
%T         =        u(1,1);     % thrust force (N)
tauroll   =        d(1,1);     % roll torque  (N*m)
taupitch  =        d(2,1);     % pitch torque (N*m)
tauyaw    =        d(3,1);     % yaw torque   (N*m)

%augmented inputs 
trhusts_diff =     u(1,1) % for the thrust variation
progress_speed_diff = u(2,1); % to compute the the progress acceleration
%semplifications
cp=cos(psi);
sp=sin(psi);
ct=cos(theta);
st=sin(theta);
cf=cos(phi);
sf=sin(phi);

%% Model equations
zdot(1,1)  =   cp*ct*xbdot+(cp*st*sf-sp*cf)*ybdot+(sp*sf+cp*cf*st)*zbdot;
zdot(2,1)  =   ct*sp*xbdot+(cf*cp+sp*st*sf)*ybdot+(cf*st*sp-cp*sf)*zbdot;
zdot(3,1)  =   -st*xbdot+ct*sf*ybdot+ct*cf*zbdot;
zdot(4,1)  =   r*ybdot-q*zbdot+g*st;
zdot(5,1)  =   -r*xbdot+p*zbdot-g*ct*sf;
zdot(6,1)  =   q*xbdot-p*ybdot-g*ct*cf+T/mass;
zdot(7,1)  =   -(sf/ct)*q+r;
zdot(8,1)  =   cf*q+sf*r;
zdot(9,1)  =   p+(sf*st/ct)*q-(cf*st/ct)*r;
zdot(10,1) =   (tauroll/Jx)-((Jz-Jy)/Jx)*r*q;
zdot(11,1) =   (taupitch/Jy)-((Jx-Jz)/Jy)*r*p;
zdot(12,1) =   (tauyaw/Jz)-((Jy-Jx)/Jz)*p*q;

%augmented state dynamics
zdot(13,1) = z(14,1); %progress_speed
zdot(14,1) = u(2,1); %progress_acceleration
zdot(15,1) = u(1,1); %thrust_speed
