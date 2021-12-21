%cost_function
function cost = cost_lenght(x,th)
x_in_kp1 = th(1);
x_out_kp1 = th (2);
x_in_k = th(3);
x_out_k = th(4);
y_in_kp1 = th(5);
y_out_kp1 = th (6);
y_in_k = th(7);
y_out_k = th(8); 

f1 = (x_in_kp1+ x(2,1)*(x_out_kp1- x_in_kp1)-(x_in_k+ x(1,1)*(x_out_k-x_in_k)))^2;
f2 = (y_in_kp1+ x(2,1)*(y_in_kp1-y_out_kp1)-(y_in_k+ x(1,1)*(y_out_k-y_in_k)))^2;
cost = f1+ f2;

end