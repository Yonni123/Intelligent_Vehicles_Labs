%
% Odometry with Khepera Mini Robot
%
% Ola Bengtsson, Bj?rn ?strand
%

clear all;
close all;

% TASK SETTINGS
SamplingRate = 1;   % Seldom of the sampling (1 uses all sample points, 10 uses every 10th point)

% %%% Khepera settings 
WHEEL_BASE = 45;                % [mm]
WHEEL_DIAMETER = 14;          % [mm]
PULSES_PER_REVOLUTION = 600;    %
MM_PER_PULSE = 15.3*pi/600 ;               % [mm / pulse]
MM_PER_PULSE = WHEEL_DIAMETER*pi/PULSES_PER_REVOLUTION; % You should write the correct one, which replaces the one above!


% %%% Uncertainty settings, which are be the same for the left and right encoders
SIGMA_WHEEL_ENCODER = 0.5/12;   % The error in the encoder is 0.5mm / 12mm travelled
% Use the same uncertainty in both of the wheel encoders
SIGMAl = SIGMA_WHEEL_ENCODER;
SIGMAr = SIGMA_WHEEL_ENCODER;


% Load encoder values
ENC = load('khepera.txt');


% Transform encoder values (pulses) into distance travelled by the wheels (mm)
Dr = ENC(1:SamplingRate:end,2) * MM_PER_PULSE;
Dl = ENC(1:SamplingRate:end,1) * MM_PER_PULSE;
N = max(size(Dr));

% Init Robot Position, i.e. (0, 0, 90*pi/180) and the Robots Uncertainty
X(1) = 0;
Y(1) = 0;
A(1) = 90*pi/180;
P(1,1:9) = [1 0 0 0 1 0 0 0 (1*pi/180)^2];

% Run until no more encoder values are available
disp('Calculating ...');
figure
for kk=2:N,
    % Change of wheel displacements, i.e displacement of left and right wheels
    dDr = Dr(kk) - Dr(kk-1);
    dDl = Dl(kk) - Dl(kk-1);
    
    % Calculate the change in X and Y (World co-ordinates)
    dX = (dDr+dDl)/2 * cos(A(kk-1)+(dDr-dDl)/(2*WHEEL_BASE));   % You should write the correct one, which replaces the one above!
    dY = (dDr+dDl)/2 * sin(A(kk-1)+(dDr-dDl)/(2*WHEEL_BASE));   % You should write the correct one, which replaces the one above!
    dA = (dDr-dDl)/(WHEEL_BASE);

    % Predict the new state variables (World co-ordinates)
    X(kk) = X(kk-1) + dX;
    Y(kk) = Y(kk-1) + dY;
    A(kk) = mod(A(kk-1) + dA, 2*pi);
    
    % Predict the new uncertainty in the state variables (Error prediction)
    Cxya_old = [P(kk-1,1:3);P(kk-1,4:6);P(kk-1,7:9)];   % Uncertainty in state variables at time k-1 [3x3] 

    dD = (dDr+dDl)/2;   % called dS in the book
    dT = (dDr-dDl)/WHEEL_BASE;
    a = mod(A(kk-1) + dT/2, 2*pi);  % The angle T + dT/2
    b = WHEEL_BASE;

    Jxold = [1  0   -dD*sin(a);      % Jacobian of X(k-1)
             0  1   dD*cos(a);
             0  0   1];

    Kr = 0.1; Kl = 0.1; % Constants
    Erl = [Kr*abs(dDr)      0;  % Variance in left and right wheel
                0       Kl*abs(dDl)];

    Jrl = [1/2 * cos(a)-dD/(2*b)*sin(a)    1/2 * cos(a)+dD/(2*b)*sin(a);
           1/2 * sin(a)+dD/(2*b)*cos(a)    1/2 * sin(a)-dD/(2*b)*cos(a);
                        1/b                            -1/b];

    Jb = [(dDr+dDl)/2 * (dDr-dDl)/(2*b^2) * sin(A(kk-1) + (dDr-dDl)/(2*b))
          -(dDr+dDl)/2 * (dDr-dDl)/(2*b^2) * cos(A(kk-1) + (dDr-dDl)/(2*b))
          -(dDr-dDl)/(b^2)];

    Eb = 0.01;   % Variance of the wheelbase (1/100)

    Cxya_new = Jxold*Cxya_old*Jxold' + Jrl*Erl*Jrl' + Jb*Eb*Jb';
    
    % Store the new co-variance matrix
    P(kk,1:9) = [Cxya_new(1,1:3) Cxya_new(2,1:3) Cxya_new(3,1:3)];
    
    % Plotting movement
    plot(X,Y,'k.'); % plot path
    plot_khepera([X(kk);Y(kk);A(kk)], WHEEL_DIAMETER, WHEEL_BASE, 3);
    drawnow();
end

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
