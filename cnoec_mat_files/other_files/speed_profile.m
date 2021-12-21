function []  = speed_profile()
% it is possible to define the lap time or also make a tracking algoritm
% type considering the value of a possible lap time and set the parameters
% to obtain that.
% lap time 
t_lap =0 ;  % total length / speed  
% to have the lower one we have to maximize the speed 
% algorithm to get the maximum speed following lateral and longidutinal
% acceleration constraints
% function intialization
delta_arc_km1 =0;
%initial guess
a_y_max =0;
a_x_max =0;
curvature_km1 = 0;
v_km1_2nd = 0 ;
v_first = zeros(norm) % it is possible to set how we want
v_k_first_run = sqrt((a_y_max/(abs(curvature_k))));

% second run 
%ciclo for k= 1....N 
v_2nd = zeros(lenght(v_first));
v_0_2nd = v_0_frst;
a_y_km1= ((v_km1_2nd)^2)*curvature_km1;
a_x_km1 = a_x_max*sqrt(1-((a_y_km1/a_y_max))^2)
v_k_guess_2nd = sqrt(((v_km1_2nd)^2)+2*a_x_km1*delta_arc_km1)
v_k_2nd = min(v_k_first_run,v_k_guess_2nd);

% third run
% for i = N-1....0, 
% initialization 
v_3rd = zeros(lenght(v_2nd));
v_N_3rd = v_3rd(N);
v_n_3rd = v_n_2nd
a_y_km1= ((v_km1_3rd)^2)*curvature_km1;
a_x_km1 = a_x_max*sqrt(1-((a_y_km1/a_y_max))^2)
v_k_guess_2rd = sqrt(((v_km1_2rd)^2)+2*a_x_km1*delta_arc_km1)
v_k_2rd = min(v_k_first_run,v_k_guess_2rd);

% optimality and feasibility 
v_first(0) = v_3rd(0);
%true when 
k0 = argmax (curvature_k);
v0 = sqrt(a_y_max/curvature(k0));



end