clear all
close all
clc

parameters.l        =   0.3;
parameters.m        =   0.2;
parameters.M        =   0.5;
parameters.beta     =   0.1;
parameters.g        =   9.81;

Ts              =   0.01;
N               =   2/Ts;
U               =   zeros(N,1);
U(1:50,1)       =   2;
U(51:150,1)    =   -2;
z0              =   [0;0;pi;0];

zsim = Pendulum_sim(U,z0,Ts,parameters);

for ind=1:5:N+1
    figure(2),grid on
    plot([zsim((ind-1)*4+1,1) zsim((ind-1)*4+1,1)-parameters.l*sin(zsim((ind-1)*4+3,1))],...
        [0,parameters.l*cos(zsim((ind-1)*4+3,1))],'-k*','linewidth',1.5)
    title(['Time: ' num2str((ind-1)*Ts) ' s'])
    axis([-2 2 -0.5 0.5]);
    pause(1e-4);
end

figure,plot(0:Ts:(N-1)*Ts,U)
figure,plot(0:Ts:N*Ts,zsim(1:4:end,1))
figure,plot(0:Ts:N*Ts,zsim(3:4:end,1))