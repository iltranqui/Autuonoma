function [F,gradF] = Rosen_F(x,a,b)
% ROSEN_F computes F(x) such that the Rosenbrock function is
% F(x)'*F(x)=(a-x_1)^2+b*(x_2-x_1^2)^2, as well as the graident of F at a
% given point x

F       =       [a-x(1,1);
                sqrt(b)*(x(2,1)-x(1,1)^2)];

gradF   =       [-1 -2*sqrt(b)*x(1,1);
                0 sqrt(b)];
    
end

