%
% Dead Reckoning with Snowhite Steed-drive Robot
%
% Björn Åstrand
%

clear all;
close all;

% %%% Snowhite settings 
L = 680;               % [mm] wheelbase
T = 0.050;             % [sec] Sampling time

% Load encoder values
CONTROL = load('snowhite.txt');

% Init Robot Position, i.e. from Ground Truth
X(1) = CONTROL(1,3);
Y(1) = CONTROL(1,4);
A(1) = CONTROL(1,5);
P(1,1:9) = [1 0 0 0 1 0 0 0 (1*pi/180)^2];

syms symv syma symT symx symy theta symL
X_k = [symx + symv*cos(syma)*symT*cos(theta+(symv*sin(syma)*symT)/(2*symL));
       symy + symv*cos(syma)*symT*sin(theta+(symv*sin(syma)*symT)/(2*symL));
       theta + (symv*sin(syma)*symT)/symL];
j_x = jacobian(X_k, [symx, symy, theta]); % J_{X_{k-1}}
j_vat = jacobian(X_k, [symv, syma, symT]); % J_{vaT}

% Run until no more values are available, i.e. speed and steering angle
N = max(size(CONTROL)); 
disp('Calculating ...');
total_dist = 0;
for kk=2:N  
    % Read current control values
    v = CONTROL(kk-1,1); % Speed of the steering wheel
    a = CONTROL(kk-1,2); % Angle of the steering wheel
    total_dist = total_dist+T*v;
    
    % Change of relative movements
    %dD = 0;
    %dA = 0;
    dD = v*cos(a)*T;   % You should write the correct one, which replaces the one above!
    dA = (v*sin(a)*T)/L;   % You should write the correct one, which replaces the one above!
    
    % Calculate the change in X and Y (World co-ordinates)
    %dX = 1;
    %dY = 1;
    dX = dD*cos(A(kk-1)+(dA/2));   % You should write the correct one, which replaces the one above!
    dY = dD*sin(A(kk-1)+(dA/2));   % You should write the correct one, which replaces the one above!
    
    % Predict the new state variables (World co-ordinates)
    X(kk) = X(kk-1) + dX;
    Y(kk) = Y(kk-1) + dY;
    A(kk) = mod(A(kk-1) + dA, 2*pi);
    
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
    
    % Plotting movement
    plot(X,Y,'b.'); hold on; % plot path
    plot(CONTROL(1:kk-1,3), CONTROL(1:kk-1,4),'k.'); % plot path
    % Plot robot drivning Dead reckoning path
    plot_threewheeled([X(kk);Y(kk);A(kk)], 100, 612, 2, a, 150, 50, L); 
    % Plot robot drivning Ground Truth path
    plot_threewheeled([CONTROL(kk-1,3);CONTROL(kk-1,4);CONTROL(kk-1,5)], 100, 612, 2, a, 150, 50, L);
    drawnow();
end


disp('Plotting ...');

% Plot the path taken by the robot, by plotting the uncertainty in the current position
figure; 
    plot(X, Y, 'b.'); hold on; % By dead reconing 
    plot(CONTROL(:,3),CONTROL(:,4),'k.'); % Ground Truth
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

% After the run, plot the results (X,Y,A), i.e. the estimated positions 
figure;
    subplot(3,1,1); plot(X, 'b'); title('X [mm]');
    subplot(3,1,2); plot(Y, 'b'); title('Y [mm]');
    subplot(3,1,3); plot(A*180/pi, 'b'); title('A [deg]');

% Plot the estimated variances (in X, Y and A) - 1 standard deviation
subplot(3,1,1); hold on;
    plot(X'+sqrt(P(:,1)), 'b:');
    plot(X'-sqrt(P(:,1)), 'b:');
hold off;
subplot(3,1,2); hold on;
    plot(Y'+sqrt(P(:,5)), 'b:');
    plot(Y'-sqrt(P(:,5)), 'b:');
hold off;
subplot(3,1,3); hold on;
    plot((A'+sqrt(P(:,9)))*180/pi, 'b:');
    plot((A'-sqrt(P(:,9)))*180/pi, 'b:');
hold off;

% Plot the errors
figure;hold on;
    subplot(3,1,1); plot(X.' - CONTROL(:,3), 'b'); title('X error [mm]');
    subplot(3,1,2); plot(Y.' - CONTROL(:,4), 'b'); title('Y error [mm]');
    subplot(3,1,3); plot(angdiff(A*180/pi,CONTROL(:,5).'*180/pi), 'b'); title('A error [deg]');
hold off;

Epos = sqrt((X.' - CONTROL(:,3)).^2 + (Y.' - CONTROL(:,4)).^2);
figure;hold on;
    plot(Epos, 'b:'); title('Euclidean distance error [mm]');
hold off;


disp('Length of track (mm)');
disp(total_dist);
disp('Travel Time (s):');
disp(T*size(CONTROL, 1))


