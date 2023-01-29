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

% Run until no more values are available, i.e. speed and steering angle
N = max(size(CONTROL)); 
disp('Calculating ...');
for kk=2:N,  
    % Read current control values
    v = CONTROL(kk-1,1); % Speed of the steering wheel
    a = CONTROL(kk-1,2); % Angle of the steering wheel
    
    % Change of relative movements
    dD = 0;
    dA = 0;
    %dD = ??;   % You should write the correct one, which replaces the one above!
    %dA = ??;   % You should write the correct one, which replaces the one above!
    
    % Calculate the change in X and Y (World co-ordinates)
    dX = 1;
    dY = 1;
    %dX = ??;   % You should write the correct one, which replaces the one above!
    %dY = ??;   % You should write the correct one, which replaces the one above!
    
    % Predict the new state variables (World co-ordinates)
    X(kk) = X(kk-1) + dX;
    Y(kk) = Y(kk-1) + dY;
    A(kk) = mod(A(kk-1) + dA, 2*pi);
    
    % Predict the new uncertainty in the state variables (Error prediction)
    Cxya_old = [P(kk-1,1:3);P(kk-1,4:6);P(kk-1,7:9)];   % Uncertainty in state variables at time k-1 [3x3]    

    Cu =   [1 0;0 1];               % Uncertainty in the input variables [2x2]
    Axya = [1 0 0;0 1 0;0 0 1];     % Jacobian matrix w.r.t. X, Y and A [3x3]
    Au =   [0 0;0 0;0 0];           % Jacobian matrix w.r.t. dD and dA [3x2]
    %Axya = ??; % You should write the correct one, which replaces the one above!
    %Au = ??;   % You should write the correct one, which repleces the one above!
    %Cu = ??;   % You should write the correct one, which replaces the one above!
    
    % Use the law of error predictions, which gives the new uncertainty
    Cxya_new = Axya*Cxya_old*Axya' + Au*Cu*Au';
    
    % Store the new co-variance matrix
    P(kk,1:9) = [Cxya_new(1,1:3) Cxya_new(2,1:3) Cxya_new(3,1:3)];
    
    % Plotting movement
    plot(X,Y,'b.'); hold on; % plot path
    plot(CONTROL(1:kk-1,3), CONTROL(1:kk-1,4),'k.'); % plot path
    % Plot robot drivning Dead reckoning path
    % plot_threewheeled([X(kk);Y(kk);A(kk)], 100, 612, 2, a, 150, 50, L); 
    % Plot robot drivning Ground Truth path
    plot_threewheeled([CONTROL(kk-1,3);CONTROL(kk-1,4);CONTROL(kk-1,5)], 100, 612, 2, a, 150, 50, L);
    drawnow();
end;


disp('Plotting ...');

% Plot the path taken by the robot, by plotting the uncertainty in the current position
figure; 
    plot(X, Y, 'b.'); hold on; % By dead reconing 
    plot(CONTROL(:,3),CONTROL(:,4),'k.'); % Ground Truth
    title('Path taken by the robot [Wang]');
    xlabel('X [mm] World co-ordinates');
    ylabel('Y [mm] World co-ordinates');
    hold on;
        for kk = 1:5:N, % Change the step to plot less seldom, i.e. every 5th
            C = [P(kk,1:3);P(kk,4:6);P(kk,7:9)];
            plot_uncertainty([X(kk) Y(kk) A(kk)]', C, 1, 2);
        end;
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
