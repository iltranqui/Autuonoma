clear all
clc

%{
%% define reference points
r = 75; % radius of circle
theta = 0:0.01:2*pi;
x1 = -r*cos(theta); % We wanted to create a skidpad track, hence we have used circle equations. 
%Alternatively, you can also use figure eight equation to generate the reference points.
y1 = r*sin(theta);
x2 = -2*r+r*cos(theta);
y2 = r*sin(theta);
xEight = [x1 x1 x2 x2];
yEight = [y1 y1 y2 y2];
% plot(xEight,yEight)
xRef = yEight;
yRef = xEight;

% calculate distance vector
rr = [yEight;xEight;xEight*0]';
distancematrix = squareform(pdist(rr));
distancesteps = zeros(length(rr)-1,1);
for i = 2:length(rr)
    distancesteps(i-1,1) = distancematrix(i,i-1);
end
totalDistance = sum(distancesteps); % Total traveled distance
distbp = cumsum([0; distancesteps]); % Distance for each waypoint
gradbp = linspace(0,totalDistance,2500); % Linearize distance

% linearize X and Y vectors based on distance
xRef2 = interp1(distbp,xRef,gradbp,'pchip');
yRef2 = interp1(distbp,yRef,gradbp,'pchip');
yRef2s = smooth(gradbp,yRef2);
xRef2s = smooth(gradbp,xRef2);

% calculate curvature vector
curvature = getCurvature(xRef2,yRef2);

%}
%% load the scene data files
% load data from Driving Scenario Designer
%load('USHighway.mat');
dataTrack = load('LVMS_ORC_NV.mat'); %Las Vegas Motor Speedway - Outside Road Course - North Variant 

%refPose = data.ActorSpecifications(1,46).Waypoints;
% define data for velocity lookup table
%lookUpt = readmatrix('velocityDistributionHighway.xlsx');
%xlt = lookUpt(2:42,1);
%ylt = lookUpt(1,2:31);
%vel = lookUpt(2:42,2:31)*4/5;
% specify simulation stop time
Tend = 45*5/4;

%% define reference points
%xRef = refPose(:,1);
%yRef = -refPose(:,2);
xRef = dataTrack.Inside(:,[1]);
yRef=  dataTrack.Inside(:,[2]);
refPose(:,1) = xRef;
refPose(:,2) = yRef;

%{ 
outside points
x_out= data_track.Outside(:,[1]);
y_out= data_track.Outside(:,[2]);
%}
%% define quad parameters used in the models
X_o = xRef(1); % initial vehicle position
Y_o = yRef(1); % initial vehicle position 
psi_o = 88.5*(pi/180); % it's an important step to initialize yaw angle

%% calculating reference pose vectors

%distance = sqrt(xRef'*xRef-yRef'*yRef);
% calculate distance vector
distancematrix = squareform(pdist(refPose));
distancesteps = zeros(length(refPose)-1,1);
for i = 2:length(refPose)
    distancesteps(i-1,1) = distancematrix(i,i-1);
end
totalDistance = sum(distancesteps); % Total traveled distance
distbp = cumsum([0; distancesteps]); % Distance for each waypoint
gradbp = linspace(0,totalDistance,100); % Linearize distance

% linearize X and Y vectors based on distance
xRef2 = interp1(distbp,xRef,gradbp,'pchip');
yRef2 = interp1(distbp,yRef,gradbp,'pchip');
yRef2s = smooth(gradbp,yRef2);
xRef2s = smooth(gradbp,xRef2);

% calculate curvature vector
curvature = getCurvature(xRef2,yRef2);

%% define reference time for plotting 
s = size(xRef);
tRef = linspace(0,Tend,length(gradbp)); % this time variable is used in the "2D Visulaization" for plotting the reference points

figure(5),plot(yRef2s,xRef2s,'b',yRef,xRef,'r',yRef2,xRef2,'g'),grid on, hold on,xlabel('y'),ylabel('x')
legend('ref2s','ref','ref2')
hold off
xq = distbp;
 

%figure(5),plot(y_in,x_in,'b',y_out,x_out,'r',y_cen,x_cen,'g'),grid on, hold on,xlabel('y'),ylabel('x')
%legend('x_in','x_out','x_cen')
%{
For the MATLAB function: spline, use the syntax: pp = spline(x,y)
 to get a structure data. Then extract the coefficients of the cubic 
spline arranged per row in a matrix form: C = pp.coefs, where C rows are:
 [a b c d] of the equation: f(x) = (a(x-x1)^3)+(b(x-x1)^2)+(c(x-x1))+(d)
 in the interval: [x1 x2].

%}
PP = spline(xRef2s,yRef2s);
C = PP.coefs;
%f(x) = (a(x-x 1)^3)+(b(x-x1)^2)+(c(x-x1))+(d);
%% Curvature Function

function curvature = getCurvature(xRef,yRef)
% Calculate gradient by the gradient of the X and Y vectors
DX = gradient(xRef);
D2X = gradient(DX);
DY = gradient(yRef);
D2Y = gradient(DY);
curvature = (DX.*D2Y - DY.*D2X) ./(DX.^2+DY.^2).^(3/2);
end
