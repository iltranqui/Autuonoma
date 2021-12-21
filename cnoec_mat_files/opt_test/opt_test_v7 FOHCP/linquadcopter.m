function [dzdot]=linquadcopter(t,dz,du,th)
%% Read parameters, states and inputs
% Parameters
mass = th(1,1);
Jx = th(2,1);
Jy = th(3,1);
Jz = th(4,1);
g = th(5,1);

% States
dX        =       dz(1,1);    % inertial X position (m)
dY        =       dz(2,1);    % inertial Y position (m)
dZ        =       dz(3,1);    % inertial Z position (m)

dxbdot    =       dz(4,1);    % body x velocity (m/s)
dybdot    =       dz(5,1);    % body y velocity (m/s)
dzbdot    =       dz(6,1);    % body z velocity (m/s)

dphi      =       dz(7,1);    % roll angle   (rad)
dtheta    =       dz(8,1);    % pitch angle (rad)
dpsi      =       dz(9,1);    % yaw angle  (rad)

dp        =       dz(10,1);   % inertial p rate (rad/s)
dq        =       dz(11,1);   % inertial q rate (rad/s)
dr        =       dz(12,1);   % inertial r rate (rad/s)

% Inputs
dT         =        du(1,1);     % thrust force (N)
dtauroll   =        du(2,1);     % roll torque  (N*m)
dtaupitch  =        du(3,1);     % pitch torque (N*m)
dtauyaw    =        du(4,1);     % yaw torque   (N*m)

%% Model equations
dzdot(1,1)  =   dxbdot;
dzdot(2,1)  =   dybdot;
dzdot(3,1)  =   dzbdot;
dzdot(4,1)  =   g*dtheta;
dzdot(5,1)  =   -g*dphi;
dzdot(6,1)  =   dT/mass;
dzdot(7,1)  =   dp;
dzdot(8,1)  =   dq;
dzdot(9,1)  =   dr;
dzdot(10,1) =   dtauroll/Jx;
dzdot(11,1) =   dtaupitch/Jy;
dzdot(12,1) =   dtauyaw/Jz;
