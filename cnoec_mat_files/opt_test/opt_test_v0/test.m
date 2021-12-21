function    [fun,fxk,gradient,xstar]    =   test(x,x_in,x_out,method,dx,x0,myoptions,ind)
% Test function to evaluate accuracy of Jacobian computation
%{
test        =   zeros(3,1);
test(1,1)   =   x(1,1)^2+5*x(2,1)^3;
test(2,1)   =   1e-3*exp(x(1,1))/x(2,1)^2;
test(3,1)   =   sin(x(1,1))*cos(x(2,1))^2;

Jtest       =   [2*x(1,1) 1e-3*exp(x(1,1))/x(2,1)^2 cos(x(1,1))*cos(x(2,1))^2;
                15*x(2,1)^2 -2*1e-3*exp(x(1,1))/x(2,1)^3 -2*sin(x(1,1))*...
                cos(x(2,1))*sin(x(2,1))];
%}
% x(1,1) = alfa of point k
% x(2,1)= alfa at point k+1
fun           =    (x_in(ind+1)+ x(2,1)*(x_out(ind+1)- x_in(ind+1))-(x_in(ind)+ x(1,1)*(x_out(ind)-x_in(ind))))^2 ....
                  +(y_in(ind+1)+ x(2,1)*(y_in(ind+1)-y_out(ind))-(y_in(ind)+ x(1,1)*(y_out(ind)-y_in(ind))))^2;
            
[fxk,gradient]     =    mygradient(@(x)test(x),xk,method,dx)

%% Gauss-Newton
% Initialize solver options
myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'GN';
myoptions.GN_funF       =	@(x)Rosen_F(x,a,b);
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-12;
myoptions.xsequence     =	'on';

% Run solver
[xstar,fxstar,niter,exitflag,xsequence] = myfminunc(@(x)test(x,x_in,x_out,method,dx,x0,myoptions,ind),x0,myoptions);

% Results
[xstar [a;a^2]]
figure,contour(x1vals,x2vals,log10(fvals),20),grid on,hold on
plot(xsequence(1,:),xsequence(2,:),'*-k'),title('Gauss-Newton')