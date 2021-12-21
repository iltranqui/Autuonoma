function [opt_trajectory_data, opt_trajectory]=quadcopter_opt_trajectory(data_track,th_opt_trajectory)
%{
TO DO
rincotrollare la funzione index curvature, mi esce una valore molto basso
mettere il calcolo della somma degli index in forma matriciale per definire
i valori di alfa per essere ottimizzati.
%}

%{
    SAVARESI EXAMPLE
input data
track: Las Vegas Motor Speedway - Outside Road Course - North Variant
RefrencePoint: (Lat, Lon, Alt) of (0,0,0) point of Cartesian coordinate system
    Inside: [x,y,z] coordinates of inner bound
    Outside: [x,y,z] coordinates of outerbound
    Centre: [x,y] coordinates of centerline
    Racing: [x,y] coordinates of racing line

%}
%   def of x_k, y_k 
%{
x_k= x_k_in + alfa_k*(x_k_out-x_k_in);
y_k = y_k_in + alfa_k*(y_k_out-x_k_in);
% limit on alfa_k
delta_alfa_k = alfa_kp1 - alfa_k; % delta tra i coefficienti delle traiettorie
%alfaK between delta_alfa_k, 1 - delta_alfa_k;

% lenght cost function 
delta_space_sqr_k = (x_kp1-x_k)^2 - (y_kp1-y_k)^2;
index_lenght = 0; % sum of delta space squared

% curvature cost function 
delta_theta_k = arctan((y_kp1-y_k)/(x_kp1-x_k)) - arctan((y_k-y_km1)/(x_k-x_km1)); % x_km1 = x(k-1)
delta_arc_k = sqrt((x_kp1-x_k)^2 + (y_kp1-y_k)^2);
curvature_k = delta_theta_k / delta_arc_k;
index_curvature = 0; % sum of curvature squared

% index curvature_lenght 
weight_index_lenght = 0;
weight_index_curvature= 0; 
index_curvature_lenght = weight_index_curvature*index_curvature + weight_index_lenght*index_lenght;
%}

% IMPLEMENTATION ON OUR PROBLEM
%% parameters
%inside points
x_in = data_track.Inside(:,[1]);
y_in= data_track.Inside(:,[2]);

%outside points
x_out= data_track.Outside(:,[1]);
y_out= data_track.Outside(:,[2]);

% initialization of coefficients alfa to be optimazed
N_points = length(x_in);
alfa = zeros(N_points); %optimal parameter to be selected for the optimal trajectory if all == 0 x_cen = x_in and y_cen = y_in

%% center point
for ind = 1: N_points
x_cen(ind)= x_in(ind) + alfa(ind,[1])*(x_out(ind)-x_in(ind));
y_cen(ind) =y_in(ind) + alfa(ind,[2])*(y_out(ind)-x_in(ind));
end


% limit on alfa_k
%delta_alfa_k = alfa_kp1 - alfa_k; % delta tra i coefficienti delle traiettorie
%alfaK between delta_alfa_k, 1 - delta_alfa_k;

%% lenght cost function 
index_length= 0;
for ind = 1 : N_points-1
delta_space_2_pow(ind) = (x_cen(ind+1)-x_cen(ind))^2 + (y_cen(ind+1)-y_cen(ind))^2; %_2_pow = al quadrato
index_length = index_length + delta_space_2_pow(ind); % sum of delta space squared

end

%% curvature cost function
index_curvature = 0;
for ind= 2 : N_points-1 % ind = 2 because if ind =1 ind-1 = 0 not exist, same for N_points-1 because if ind = N_point, ind = N_points +1 not exist
delta_theta(ind) = atan((y_cen(ind+1)-y_cen(ind))/(x_cen(ind+1)-x_cen(ind)))...
                 - atan((y_cen(ind)-y_cen(ind-1))/(x_cen(ind)-x_cen(ind-1))); % x_km1 = x(k-1)
             
delta_arc(ind) = sqrt((x_cen(ind+1)-x_cen(ind))^2 + (y_cen(ind+1)-y_cen(ind))^2);
curvature(ind) = delta_theta(ind) / delta_arc(ind);
index_curvature = index_curvature + curvature(ind)^2 ;% sum of curvature squared
end 
tvec_FFD    =       1:1:N_points-1;

%% set the weights for the indexes
weight_index_curvature=th_opt_trajectory(1);
weight_index_lenght = th_opt_trajectory(2); 
index_curvature_length = weight_index_curvature * index_curvature + weight_index_lenght*index_length;


%{ 
%% optimization cvx

for ind = 1 : N_points-1
alfa= alfa(ind); %alfa(k)

%alfa(k+1)
%minimize 
 (x_in(ind+1)+alfa_1*(x_out(ind+1)- x_in(ind+1))-(x_in(ind)+alfa*(x_out(ind)-x_in(ind))))^2 ....
    +(y_in(ind+1)+alfa_1*(y_in(ind+1)-y_out(ind))-(y_in(ind)+alfa(y_out(ind)-y_in(ind))))^2;


alfa(ind+1) = alfa_1;
end

%}
%% outputs
opt_trajectory(:,1) = x_cen;
opt_trajectory(:,2) = y_cen;

opt_trajectory_data(1) = index_curvature; 
opt_trajectory_data(2)= index_length;
opt_trajectory_data(3)= index_curvature_length;



%% per la stampa dei grafici 
 
figure(1),plot(delta_arc),grid on, hold on,ylabel('delta_arc');
figure(2),plot(curvature),grid on, hold on,ylabel('delta_curvature');
%figure(3),plot(tvec_FFD,index_curvature),grid on, hold on,xlabel('Time (s)'),ylabel('index_curvature')
% index curvature length
%figure(4),plot(y_in,x_in),grid on, hold on,xlabel('y_in)'),ylabel('c_in')
%figure(4),plot(y_out,x_out),grid on, hold on,xlabel('y_out)'),ylabel('c_out')
%figure(4),plot(y_cen,x_cen),grid on, hold on,xlabel('y_cen)'),ylabel('c_cen')
figure(5),plot(y_in,x_in,'b',y_out,x_out,'r',y_cen,x_cen,'g'),grid on, hold on,xlabel('y'),ylabel('x')
legend('x_in','x_out','x_cen')
hold off
figure(6),plot(y_in,x_in,'b'),grid on, hold on,xlabel('y'),ylabel('x') %solo per vedere x_in,x_inperchè ora x_cen = x_in adn y_cen = y_in
legend('x_in')
hold off


