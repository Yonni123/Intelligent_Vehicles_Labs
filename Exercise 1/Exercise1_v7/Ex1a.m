% Stationary GPS receiver 
% (c) Björn Åstrand, v3
close all; clear all; 

DATA = load('gps_ex1_window.txt');
Longitude = DATA(:,4); % read all rows in column 4
Latitude  = DATA(:,3); % read all rows in column 3
figure, plot(Longitude,Latitude);
title('Position in NMEA-0183 format');
xlabel('Longitude');
ylabel('Latitude');

% 3.1 Write a function that transform your longitude and latitude angles 
% from NMEA-0183 into meters
% 1. longitude and latitude angles from NMEA-0183 into degrees  

LongDeg = floor(Longitude/100) + (Longitude - floor(Longitude/100)*100)/60;
LatDeg = floor(Latitude/100) + (Latitude - floor(Latitude/100)*100)/60;

figure, plot(LongDeg,LatDeg);
title('Position in degrees');
xlabel('Longitude');
ylabel('Latitude');

% 2. longitude and latitude angles from NMEA-0183 into degrees
F_lon = 62393; % from table
F_lat = 111342; % from table

X = F_lon * LongDeg;
Y = F_lat * LatDeg;

figure, plot(X,Y);
title('Position in meters');
xlabel('Longitude');
ylabel('Latitude');

% 3.2 Estimate the mean and variance of the position (in x and y)
% Matlab fuctions mean() and var()
% -> Your code here

% 3.3 Plot, with respect to time, the errors and the auto-correlation 
% in x and y separately.

% -> Your code here

