function [f,gradient,hessf] = opt(x,x_in_kp1,x_in_k,x_out_kp1,x_out_k,y_in_kp1,y_in_k,y_out_kp1,y_out_k)
% ROSEN computes the Rosenbrock function f(x)=(a-x_1)^2+b*(x_2-x_1^2)^2, as
% well as its gradient and Hessian at the given point x

f1 = (x_in_kp1+ x(2,1)*(x_out_kp1- x_in_kp1)-(x_in_k+ x(1,1)*(x_out_k-x_in_k)))^2;
f2 = (y_in_kp1+ x(2,1)*(y_in_kp1-y_out_kp1)-(y_in_k+ x(1,1)*(y_out_k-y_in_k)))^2;
f = f1+ f2;
%[f] =(x_in_kp1+ x(2)*(x_out_kp1- x_in_kp1)-(x_in_k+ x(1)*(x_out_k-x_in_k)))^2 +(y_in_kp1+ x(2)*(y_in_kp1-y_out_kp1)-(y_in_k+ x(1)*(y_out_k-y_in_k)))^2;
%f       =       [a-x(1,1);
    %            sqrt(b)*(x(2,1)-x(1,1)^2)];
%f =(x_in_kp1+ x*(x_out_kp1 - x_in_kp1)-(x_in_k + y*(x_out_k-x_in_k)))^2 +(y_in_kp1+ x*(y_in_kp1-y_out_kp1)-(y_in_k+ y*(y_out_k-y_in_k)))^2;
%xk   =   [1;1];
%[fxk,gradient] =  mygradient(@(x)opt(x,x_in_kp1,x_in_k,x_out_kp1,x_out_k,y_in_kp1,y_in_k,y_out_kp1,y_out_k),xk,'FD',2^-26)

%gradf = gradient
%hessf = 0;
%Hessf = mygradient(gradf,xk,'FD',2^-26)
end

