%
% Main script that reads controller data and laser data

clear all;
close all;

% Co-ordinates of the ref. nodes
REF = [1920 9470;   % 01
       10012 8179;  % 02
       9770 7590;   % 03
       11405 7228;  % 04
       11275 6451;  % 05
       11628 6384.5;% 06
       11438 4948;  % 07
       8140 8274;   % 08
       8392 8486;   % 09
       3280 2750;   % 10
       7250 2085;   % 11
       9990 1620;   % 12
       7485 3225;   % 13
       9505 3893;   % 14
       9602 4278;   % 15
       10412 4150;  % 16
       4090 7920;   % 17
       8010 5290;   % 18
       8255 6099;   % 19
       7733 6151;   % 20
       7490 6136;   % 21
       7061 5420;   % 22
       7634 5342];  % 23

% LINE SEGMENT MODEL (Indeces in the REF-vector)
LINES = [1 8;       % L01
         9 2;       % L02
         2 3;       % L03
         3 4;       % L04
         4 5;       % L05
         5 6;       % L06
         6 7;       % L07
         17 10;     % L08
         10 12;     % L09
         11 13;     % L10
         12 16;     % L11
         16 15;     % L12
         15 14;     % L13
         19 21;     % L14
         22 18;     % L15
         20 23];    % L16
         
% Control inputs (velocity and steering angle)
CONTROL = load('control_joy.txt');

% Laser data
LD_HEAD   = load('laser_header.txt');
LD_ANGLES = load('laser_angles.txt');
LD_MEASUR = load('laser_measurements.txt');
LD_FLAGS  = load('laser_flags.txt');

[no_inputs co] = size(CONTROL);

% Robots initial position
X(1) = CONTROL(1,4);
Y(1) = CONTROL(1,5);
A(1) = CONTROL(1,6);
P(1,1:9) = [1 0 0 0 1 0 0 0 (1*pi/180)^2];

syms symv syma symT symx symy theta symL
X_k = [symx + symv*cos(syma)*symT*cos(theta+(symv*sin(syma)*symT)/(2*symL));
       symy + symv*cos(syma)*symT*sin(theta+(symv*sin(syma)*symT)/(2*symL));
       theta + (symv*sin(syma)*symT)/symL];
j_x = jacobian(X_k, [symx, symy, theta]); % J_{X_{k-1}}
j_vat = jacobian(X_k, [symv, syma, symT]); % J_{vaT}

scan_idx = 1;
fig_path = figure;
fig_env = figure;
ScanPosIndx = [];
N = max(size(CONTROL));
% Plot the line model
figure(fig_env); plot_line_segments(REF, LINES, 1);
     
for kk = 2:no_inputs,
    % Check if we should get a position fix, i.e. if the time stamp of the
    % next laser scan is the same as the time stamp of the control input
    % values
    if LD_HEAD(scan_idx,1) == CONTROL(kk-1,1),
        % Mark the position where the position fix is done - and the size
        % of the position fix to be found
        figure(fig_path);
        hold on; plot(X(kk-1), Y(kk-1), 'ro'); hold off;
        hold on; plot(CONTROL(kk-1,4), CONTROL(kk-1,5), 'ro'); hold off;
        hold on; plot([X(kk-1) CONTROL(kk-1,4)], [Y(kk-1) CONTROL(kk-1,5)], 'r'); hold off;
        
        % > Log pos (index) there scan is taken
        ScanPosIndx = [ScanPosIndx kk-1]; % Used for plotting
        
        % Get the position fix - Use data points that are ok, i.e. with
        % flags = 0
        DATAPOINTS = find(LD_FLAGS(scan_idx,:) == 0);
        angs = LD_ANGLES(scan_idx, DATAPOINTS);
        meas = LD_MEASUR(scan_idx, DATAPOINTS);
        
        % Plot schematic picture of the Snowhite robot
        alfa = 660;
        beta = 0;
        gamma = -90*pi/180;
        figure(fig_env); plot_line_segments(REF, LINES, 1);
        plot_threewheeled_Laser([X(kk-1) Y(kk-1) A(kk-1)]', 100, 612, 2, CONTROL(kk-1,5), 150, 50, 680, alfa, beta, gamma, angs, meas, 1);
        
        % Task 3 - write your code in the function "Cox_LineFit"
        % [dx,dy,da,C] = Cox_LineFit(angs, meas, [X(kk-1) Y(kk-1) A(kk-1)]', LINEMODEL,[alfa beta gamma]);
        % Returns => Position fix + Unceratinty of the position fix
        LINEMODEL = [REF(LINES(:,1),1:2) REF(LINES(:,2),1:2)];
        [dx,dy,da,C] = Cox_LineFit(angs, meas, [X(kk-1) Y(kk-1) A(kk-1)]', LINEMODEL,[alfa beta gamma], kk-1);
        % dx is the estimated x-position error
        % dy is the estimated y-position error
        % da is the estimated angle error
        % C is the covariance matrix of the position fix
      
        % ... AND HERE ...
        % Update the position, i.e. X(kk-1), Y(kk-1), A(kk-1) and C(kk-1)
        % Here you shall add your code for the three experiments
        % Task 4 Dead Reconing + Cox Update
        %X(kk-1) = X(kk-1) + dx;
        %Y(kk-1) = Y(kk-1) + dy;
        %A(kk-1) = A(kk-1) + da;
        %figure(fig_path);
        %plot_uncertainty([X(kk-1) Y(kk-1)]', C, 1, 2);

        % Task 5a Kalman Filter with simulated position fixes (small) (Exercise 4)
        %sigma_x = 10;
        %sigma_y = 10;
        %sigma_theta = 1*pi/180;
        
        % Task 5b Kalman Filter with simulated position fixes (large) (Exercise 4)
        %sigma_x = 100;
        %sigma_y = 100;
        %sigma_theta = 3*pi/180;

        %Xpf(kk-1) = CONTROL(kk-1,4) + randn(size(CONTROL(kk-1,4)))*sqrt(sigma_x);
        %Ypf(kk-1) = CONTROL(kk-1,5) + randn(size(CONTROL(kk-1,5)))*sqrt(sigma_y);
        %Apf(kk-1) = CONTROL(kk-1,6) + randn(size(CONTROL(kk-1,6)))*sqrt(sigma_theta);
        %Error_pf = [sigma_x^2 0 0; 0 sigma_y^2 0; 0 0 sigma_theta^2];

        % Task 6 Kalmanfilterr with Cox position update (Exercise 4)
        Xpf(kk-1) = X(kk-1) + dx;
        Ypf(kk-1) = Y(kk-1) + dy;
        Apf(kk-1) = A(kk-1) + da;
        Error_pf = C;

        Error_x = [P(kk-1,1:3);P(kk-1,4:6);P(kk-1,7:9)];
        P(kk-1,1:9) = reshape((Error_pf^(-1) + Error_x^(-1))^(-1),[1,9]);
        POS = Error_pf*(Error_pf+Error_x)^(-1)*[X(kk-1); Y(kk-1); A(kk-1)] + Error_x*(Error_pf + Error_x)^(-1)*[Xpf(kk-1); Ypf(kk-1); Apf(kk-1)];
        X(kk-1) = POS(1);
        Y(kk-1) = POS(2);
        A(kk-1) = POS(3);

        % Next time use the next scan
        scan_idx = mod(scan_idx, max(size(LD_HEAD))) + 1;
    end;
    
    % Mark the estimated (dead reckoning) position
    figure(fig_path);
    hold on; plot(X(kk-1), Y(kk-1), 'b.'); hold off;
    % Mark the true (from the LaserWay system) position
    hold on; plot(CONTROL(kk-1,4), CONTROL(kk-1,5), 'k.'); hold off;
    
    % Estimate the new position (based on the control inputs) and new
    % uncertainty
    v = CONTROL(kk-1,2);
    a = CONTROL(kk-1,3);
    T = 0.050;
    L = 680;

    X(kk) = X(kk-1) + cos(a)*v*cos(A(kk-1))*T;
    Y(kk) = Y(kk-1) + cos(a)*v*sin(A(kk-1))*T;
    A(kk) = A(kk-1) + sin(a)*v*T/L;
    
    % ALSO UPDATE THE UNCERTAINTY OF THE POSITION
    % Task 2 Dead Reckoning - Add your code here

    % Predict the new uncertainty in the state variables (Error prediction)
    Cxya_old = [P(kk-1,1:3);P(kk-1,4:6);P(kk-1,7:9)];   % Uncertainty in state variables at time k-1 [3x3]
    
    Axya = subs(j_x,[symv, syma, symT, symx, symy, theta, symL],[v, a, T, X(kk-1), Y(kk-1), A(kk-1), L]);
    
    Au = subs(j_vat,[symv, syma, symT, symx, symy, theta, symL],[v, a, T, X(kk-1), Y(kk-1), A(kk-1), L]);

    sigma_v = 0.1;
    sigma_a = 0.1;
    sigma_t = 0.0001;

    Cu =   [sigma_v^2   0   0;
            0   sigma_a^2   0;
            0   0   sigma_t^2];
    
    % Use the law of error predictions, which gives the new uncertainty
    Cxya_new = Axya*Cxya_old*Axya' + Au*Cu*Au';
    
    % Store the new co-variance matrix
    P(kk,1:9) = [Cxya_new(1,1:3) Cxya_new(2,1:3) Cxya_new(3,1:3)];
    
end

ERROR = [X' Y' A'] - CONTROL(:,4:6);
ERROR(:,3) = AngDifference(A',CONTROL(:,6));
ERROR = abs(ERROR);
figure,
subplot(3,1,1);
plot(ERROR(:,1),'b'); hold;
plot(sqrt(P(:,1)),'r'); % one sigma
plot(ScanPosIndx,sqrt(P(ScanPosIndx,1)),'k.'); % one sigma
title('Error X [mm] and uncertainty [std] (red)');

subplot(3,1,2);
plot(ERROR(:,2),'b'); hold;
plot(sqrt(P(:,5)),'r'); % one sigma
title('Error Y [mm] and uncertainty [std] (red)');

subplot(3,1,3);
plot(ERROR(:,3)*180/pi,'b'); hold;
plot(sqrt(P(:,9))*180/pi,'r'); % one sigma
title('Error A [degree] and uncertainty [std] (red)');

% Plot the path taken by the robot, by plotting the uncertainty in the current position
figure; 
    plot(X, Y, 'b.'); hold on; % By dead reckoning 
    plot(CONTROL(:,4),CONTROL(:,5),'k.'); % Ground Truth
    title('Path taken by the robot [Wang]');
    xlabel('X [mm] World co-ordinates');
    ylabel('Y [mm] World co-ordinates');
    hold on;
        for kk = 1:5:N % Change the step to plot less seldom, i.e. every 5th
            C = [P(kk,1:3);P(kk,4:6);P(kk,7:9)];
            plot_uncertainty([X(kk) Y(kk) A(kk)]', C, 1, 2);
        end
    hold off;
    axis('equal');
