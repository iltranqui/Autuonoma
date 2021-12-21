function [f,gradf,Hessf] = Rosen(x,a,b)
% ROSEN computes the Rosenbrock function f(x)=(a-x_1)^2+b*(x_2-x_1^2)^2, as
% well as its gradient and Hessian at the given point x

f           =   (a-x(1,1))^2+b*(x(2,1)-x(1,1)^2)^2;

gradf       =   [-2*a+2*x(1,1)-4*b*x(2,1)*x(1,1)+4*b*x(1,1)^3;
                 2*b*(x(2,1)-x(1,1)^2)];
             
Hessf       =   [2-4*b*x(2,1)+12*b*x(1,1)^2 -4*b*x(1,1);
                 -4*b*x(1,1) 2*b];
end

