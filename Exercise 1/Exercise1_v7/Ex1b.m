% Stationary GPS receiver 
% (c) Björn Åstrand, v3
close all; clear all; 

DATA = load('gps_ex1_morningdrive.txt');
Longitude = DATA(:,4); % read all rows in column 4
Latitude  = DATA(:,3); % read all rows in column 3
figure, plot(Longitude,Latitude);
title('Position in NMEA-0183 format');
xlabel('Longitude');
ylabel('Latitude');

% 3.1 Write a function that transform your longitude and latitude angles 
% from NMEA-0183 into meters
% 1. longitude and latitude angles from NMEA-0183 into degrees  

LongDeg = 0;% = floor(Longitude/100) + (Longitude - floor(Longitude/100)*100)/60;
LatDeg = 0; % 

figure, plot(LongDeg,LatDeg);
title('Position in degrees');
xlabel('Longitude');
ylabel('Latitude');

% 2. longitude and latitude angles from NMEA-0183 into degrees
F_lon = 0; % from table
F_lat = 0; % from table

X = F_lon * LongDeg;
Y = F_lat * LatDeg;

figure, plot(X,Y);
title('Position in meters');
xlabel('Longitude');
ylabel('Latitude');

% 4.2 Calculate the maximum speed taken by the driver. 
% -> Your code here


% 4.3 Calculate the headings (and plot them with respect to time) along the path .
% -> Your code here

