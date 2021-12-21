%% clear the workspace and the command window

clear all
close all 
clc


data_track = load('LVMS_ORC_NV.mat'); %Las Vegas Motor Speedway - Outside Road Course - North Variant 
%% parameters
%inside points
x_in = data_track.Inside(:,1);
y_in= data_track.Inside(:,2);

%outside points
x_out= data_track.Outside(:,1);
y_out= data_track.Outside(:,2);

N_points = length(x_in);
m= zeros(N_points,1);
q= zeros(N_points,1);
on_line=zeros(N_points,1);

for ind = 1 : N_points
    
p1 = [x_in(ind),y_in(ind)];
p2 = [x_out(ind),y_out(ind)];
[m(ind),q(ind)] = line_m_q(p1,p2);

on_line(ind) = p_on_line(p1,m(ind),q(ind));

end


function [m,q] = line_m_q(p1,p2)
m = (p2(2)-p1(2))/ (p2(1)-p1(1));
q = -m*p1(1)+ p1(2);
end


function on_line= p_on_line(p,m,q)
%{
x_round = round(p(1));
y_round = round(p(2));
m_round = round(m);
q_round = round(q);
y_round
diff = m_round*x_round + q_round 
    if y_round == m_round*x_round +q_round 
        on_line  = 1;
    else
        on_line = 0;

    end
%}
x_round = p(1);
y_round = p(2);
m_round = m ;
q_round = q;
diff = m_round*x_round + q_round ;
 if y_round <= diff + 0.0001 
        on_line  = 1;
    else
        on_line = 0;

 end

end 

function [x,y] = p_intersection(m1,q1,m2,q2)
x = NaN;
y = NaN;
d = (-m1+m2);
    if (d~= 0)
        x = (q1-q2)/d;
        y = (-m1*q2+m2*q1)/d;
    end
end

function f = cost_time(x)
    x_p = zeros(N_points,1);
    y_p = zeros(N_points,1);
    for ind = 1 : N_points - 1

    p1 = [x_p(ind),y_p(ind)];
    p2 = [x_p(ind+1),y_p(ind+1)];
    [m1,q1] = line_m_q(p1,p2);
    p3 = [x_in(ind+1),y_in(ind+1)];
    p4 = [x_out(ind+1),y_out(ind+1)];
    [m2,q2] = line_m_q(p3,p4);

    %on_line(ind) = p_on_line(p1,m(ind),q(ind));

    [x_p(ind+1),y_p(ind+1)]= p_intersection(m1,q1,m2,q2);

    d(ind+1) = sqrt((x(ind+1)-x_p(ind))^2+(y_p(ind+1)-y_p(ind))^2); 
    f = sum(d/v);
    end

end