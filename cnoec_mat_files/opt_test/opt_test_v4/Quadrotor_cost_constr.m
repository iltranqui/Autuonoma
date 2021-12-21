function v = Quadrotor_cost_constr(x,Ts,Np,th)

%% Build vector of inputs
t_in        =   [0:Ts:(Np-1)*Ts]';
z0         =   [0;x(1:2,1);0;x(3,1);0];
u_in        =   [x(4:Np+3,1)';    % vettore accelerazioni iniziale
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
    zdot               =   vehicle(0,z_sim(:,ind-1),u,0,th); % dovrò mettere il quadrotor
    z_sim(:,ind)       =   z_sim(:,ind-1)+Ts/Nblock*zdot;
end

pX_sim       =   z_sim(1,1:Nblock:end)';
pY_sim       =   z_sim(2,1:Nblock:end)';
Vx_sim      =    z_sim(3,1:Nblock:end)';
Vy_sim      =    z_sim(4,1:Nblock:end)';
v_sim = (Vx_sim+Vy_sim)^2;

%{

%% Compute path constraints h(x)
h           =   [Y_sim-(tanh((X_sim-100)/2e1)*10+5);
                -Y_sim+(tanh((X_sim-75)/2e1)*10+15)];

%% Compute cost function f(x)
delta_diff  =   (x(Np+5:end,1)-x(Np+4:end-1,1));
Td_diff     =   (x(5:Np+3,1)-x(4:Np+2,1));
f           =   -X_sim(end,1)+1e3*z_sim(5,end)^2+1e3*(delta_diff'*delta_diff)+1e-2*(Td_diff'*Td_diff);

%% Stack cost and constraints
%}
% il vettore u è la mia x
% Compute the cost function
P = [pX_sim ; pY_sim ];
center_line = [2*cos(sqrt(distance_travelled))+2*sin(sqrt(distance_travelled))*sqrt(distance_travelled);...
                2*sin(sqrt(distance_travelled))- 2*cos(sqrt(distance_travelled))*sqrt(distance_travelled)]
tangente_Pcen = gradiente_Center_line_in_
parte1 = tangente_Pcen * P;
f = parte1;
%f = parte1 + parte2 + parte3
v           =   [f;h];
