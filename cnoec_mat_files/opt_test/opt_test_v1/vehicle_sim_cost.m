function    [cost,ysim]    =   vehicle_sim_cost(x,z0,uin,ymeas,th,Ts,Q,scaling)
% Function that computes the output trajectory of the vehicle model
% introduced in Lab. session A and the quadratic cost given by
% the sum of squared weighted errors between the simulated output
% and the measured one. Variable x corresponds to the vehicle parameters 
% to be estimated, possibly scaled. The simulation is carried out with forward finite
% differences

[err_vec,ysim]    =   vehicle_sim_err(x,z0,uin,ymeas,th,Ts,Q,scaling);

%% Compute sum of squared errors
cost    =   sum(err_vec.*err_vec);
