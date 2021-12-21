function [zsim] = Pendulum_sim(U,z0,Ts,parameters)

N=length(U);

zsim        =   zeros(4*(N+1),1);
zsim(1:4,1) =   z0;
ztemp       =   z0;

for ind=2:N+1
    ztemp                      =     ztemp+Ts*PendulumOnCart(ztemp,...
                                        U(ind-1,1),parameters);
    zsim((ind-1)*4+1:ind*4,1)  =     ztemp;        
end
 

