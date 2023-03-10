%
% Odometry with Khepera Mini Robot
%
% Ola Bengtsson, Bj?rn ?strand
%

clear all;
%close all;

% TASK SETTINGS
SamplingRate = 10;   % Seldom of the sampling (1 uses all sample points, 10 uses every 10th point)

% %%% Khepera settings 
WHEEL_BASE = 53;                % [mm]
WHEEL_DIAMETER = 15.3;          % [mm]
PULSES_PER_REVOLUTION = 600;    %
MM_PER_PULSE = 15.3*pi/600 ;               % [mm / pulse]
MM_PER_PULSE = WHEEL_DIAMETER*pi/PULSES_PER_REVOLUTION; % You should write the correct one, which replaces the one above!


% %%% Uncertainty settings, which are be the same for the left and right encoders
SIGMA_WHEEL_ENCODER = 0.5/12;   % The error in the encoder is 0.5mm / 12mm travelled
% Use the same uncertainty in both of the wheel encoders
SIGMAl = SIGMA_WHEEL_ENCODER;
SIGMAr = SIGMA_WHEEL_ENCODER;


% Load encoder values
ENC = load('khepera_circle.txt');


% Transform encoder values (pulses) into distance travelled by the wheels (mm)
Dr = ENC(1:SamplingRate:end,2) * MM_PER_PULSE;
Dl = ENC(1:SamplingRate:end,1) * MM_PER_PULSE;
N = max(size(Dr));

% Init Robot Position, i.e. (0, 0, 90*pi/180) and the Robots Uncertainty
X(1) = 0;
Y(1) = 0;
A(1) = 90*pi/180;
P(1,1:9) = [1 0 0 0 1 0 0 0 (1*pi/180)^2];

% Save robot position without compensation term
XW(1) = 0;
YW(1) = 0;

% Run until no more encoder values are available
disp('Calculating ...');
figure
for kk=2:N,
    % Change of wheel displacements, i.e displacement of left and right wheels
    dDr = Dr(kk) - Dr(kk-1);
    dDl = Dl(kk) - Dl(kk-1);
    
    % Change of relative movements
    dD = 0;
    dA = 0.017;
    dD = (dDr+dDl)/2;   % You should write the correct one, which replaces the one above!
    dA = (dDr-dDl)/WHEEL_BASE;   % You should write the correct one, which replaces the one above!

    % Save my dD and dA to an array so they can be plotted later
    dDa(kk-1) = dD;
    dAa(kk-1) = dA;
    
    % Calculate the change in X and Y (World co-ordinates)
    dX = 1;
    dY = 1;
    dX = dD * cos(A(kk-1)+(dA/2));   % You should write the correct one, which replaces the one above!
    dY = dD * sin(A(kk-1)+(dA/2));   % You should write the correct one, which replaces the one above!
    
    % Check if we should use correction term or not
    term = 1;
    if dA ~= 0
        term = sin(dA/2)/(dA/2);
    end

    % Predict the new state variables (World co-ordinates)
    X(kk) = X(kk-1) + dX*term;
    Y(kk) = Y(kk-1) + dY*term;
    A(kk) = mod(A(kk-1) + dA, 2*pi);

    XW(kk) = X(kk-1) + dX;
    YW(kk) = Y(kk-1) + dY;
    
    % Predict the new uncertainty in the state variables (Error prediction)
    Cxya_old = [P(kk-1,1:3);P(kk-1,4:6);P(kk-1,7:9)];   % Uncertainty in state variables at time k-1 [3x3]   
    a = mod(A(kk-1) + dA/2, 2*pi);  % The angle T + dT/2

    Cu =   [1 0;0 1];               % Uncertainty in the input variables [2x2]
    Axya = [1 0 0;0 1 0;0 0 1];     % Jacobian matrix w.r.t. X, Y and A [3x3]
    Au =   [0 0;0 0;0 0];           % Jacobian matrix w.r.t. dD and dA [3x2]

    Axya = [1   0   -dD*sin(a);
            0   1   dD*cos(a);
            0   0       1]; % You should write the correct one, which replaces the one above!

    Au =   [cos(a)  (-dD/2)*sin(a);
            sin(a)  (dD/2)*cos(a);
                0       1];   % You should write the correct one, which repleces the one above!

    Cu =   [(SIGMAr^2+SIGMAl^2)/4   0;
                0   (SIGMAr^2+SIGMAl^2)/WHEEL_BASE^2];   % You should write the correct one, which replaces the one above!
    
    % Use the law of error predictions, which gives the new uncertainty
    Cxya_new = Axya*Cxya_old*Axya' + Au*Cu*Au';
    
    % Store the new co-variance matrix
    P(kk,1:9) = [Cxya_new(1,1:3) Cxya_new(2,1:3) Cxya_new(3,1:3)];
    
    % Plotting movement
    plot(X,Y,'k.'); % plot path
    plot_khepera([X(kk);Y(kk);A(kk)], WHEEL_DIAMETER, WHEEL_BASE, 3);
    drawnow();
end;
CV = cov(dDa,dAa*180/pi);

disp('Plotting ...');

% Plot the path taken by the robot, by plotting the uncertainty in the current position
figure(2); 
    %plot(X, Y, 'b.');
    title('Path taken by the robot [Wang]');
    xlabel('X [mm] World co-ordinates');
    ylabel('Y [mm] World co-ordinates');
    hold on;
        for kk = 1:1:N,
            C = [P(kk,1:3);P(kk,4:6);P(kk,7:9)];
            plot_uncertainty([X(kk) Y(kk) A(kk)]', C, 1, 2);
        end;
    hold off;
    axis('equal');

% After the run, plot the results (X,Y,A), i.e. the estimated positions 
figure;

subplot(3,1,1);
plot(X, 'b');
hold on;
plot(XW, 'r');
title('X [mm]');
legend('X', 'XW');

subplot(3,1,2);
plot(Y, 'b');
hold on;
plot(YW, 'r');
title('Y [mm]');
legend('Y', 'YW');

subplot(3,1,3);
plot(A*180/pi, 'b');
title('A [deg]');

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

% After the run, plot the results (X,Y,A), i.e. the estimated positions 
figure;
    subplot(2,1,1); plot(dDa, 'b'); title('dD');
    subplot(2,1,2); plot(dAa*180/pi, 'b'); title('dA [deg]');


figure;
    diff = X-XW;
    plot(diff, 'b');