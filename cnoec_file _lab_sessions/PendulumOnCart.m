function [zdot,jacob_z,jacob_u] = PendulumOnCart(z,u,parameters)

l       =   parameters.l;
m       =   parameters.m;
M       =   parameters.M;
beta    =   parameters.beta;
g       =   parameters.g;

xdot    =   z(2,1);
th      =   z(3,1);
thdot   =   z(4,1);

Mtot    =   M+m*(1-cos(th)^2);

zdot    =   [xdot;
            (-beta*xdot-m*l*sin(th)*thdot^2+m*g*sin(th)*cos(th)+u)/Mtot;
            thdot;
            g/l*sin(th)*(1+m*cos(th)^2/Mtot)+cos(th)*...
            (-beta*xdot-m*thdot^2*l*...
            sin(th)+u)/(l*Mtot)];

jacob_z = zeros(4);

jacob_u = zeros(4,1);

end

