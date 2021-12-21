%main 
clc
clear all
R = 0.01;  % imput change weight
q = 10 ; % slack weight
Ts =10;  % time of simulation
Np = 10; % prediction horizon

th = [R;q];
v = Quadrotor_cost_constr(x,Ts,Np,th);