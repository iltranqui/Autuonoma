function [z_sim] = Vehicle_traj(x,Ts,Np,th)
% Function that computes the trajectory of the vehicle model
% introduced in Lab. session A and returns the system state together with
% plots of the relevant quantities

%% Build vector of inputs
t_in        =   [0:Ts:(Np-1)*Ts]';
z0         =   [0;x(1:2,1);0;x(3,1);0];
u_in        =   [x(4:Np+3,1)';
                x(Np+4:end,1)'];

assignin('base','z0',z0);
assignin('base','t_in',t_in);
assignin('base','u_in',u_in);


%% Run simulation with FFD
time_FFD    =   [0:0.01:(Np-1)*Ts];
Nblock      =   Ts/0.01;
Nsim_FFD    =   length(time_FFD);

z_sim      =   zeros(6,Nsim_FFD);
z_sim(:,1) =   z0;
for ind=2:Nsim_FFD
    u                   =   u_in(:,1+floor(time_FFD(ind)/Ts));
    zdot               =   vehicle(0,z_sim(:,ind-1),u,0,th);
    z_sim(:,ind)       =   z_sim(:,ind-1)+Ts/Nblock*zdot;
end

X_sim       =   z_sim(1,1:end);
Y_sim       =   z_sim(2,1:end);

%% Plot (X,Y) trajectory and constraints
Ymin        =   tanh((X_sim-100)/2e1)*10+5;
Ymax        =   tanh((X_sim-75)/2e1)*10+15;

figure(1),subplot(2,2,1),plot(X_sim,Y_sim,X_sim,Ymin,'k',X_sim,Ymax,'k'),grid on
xlabel('X (m)'),ylabel('Y (m)')
subplot(2,2,2),plot(time_FFD,z_sim(3,:)),grid on
xlabel('Time (s)'),ylabel('Longitudinal speed (m/s)')
subplot(2,2,3),plot(t_in,u_in(2,:)),grid on
xlabel('Time (s)'),ylabel('Front steering angle (rad)')
subplot(2,2,4),plot(t_in,u_in(1,:)),grid on
xlabel('Time (s)'),ylabel('Driving torque (Nm)')



