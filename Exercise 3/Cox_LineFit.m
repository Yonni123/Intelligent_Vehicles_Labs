%
% Fits (by translation and rotation) data points to a set of 
% line segments, i.e. applying the Cox's algorithm
% function [ddx,ddy,dda,C] = Cox_LineFit(ANG, DIS, POSE, LINEMODEL, SensorPose)
% Input:  ANG, DIS - the data points (in polar co-ordinates)
%         POSE - the initial pose (in world co-ordinates)
%         LINEMODEL - the line segments (in world co-ordinates)
%         SensorPose - the sensor pose (in robot co-ordinates)
% Output: ddx,ddy,dda - the translation and rotation to be applied to the initial pose
%         C - the covariance matrix
% 
function [ddx,ddy,dda,C] = Cox_LineFit(ANG, DIS, POSE, LINEMODEL, SensorPose)
    % Init variables
    ddx = 0; ddy = 0; dda = 0;  % The translation and rotation to be returned
    Rx = POSE(1,1); Ry = POSE(2,1); Ra = POSE(3,1);     % The initial pose (in world co-ordinates)
    Sx = SensorPose(1); Sy = SensorPose(2); Sa = SensorPose(3);   % X, Y and Theta of the sensor location (in robot co-ordinates)
    max_iterations = 100; % <------YOU NEED TO CHANGE THIS NUMBER
    no_update = 0;
    
    % Step 0 - Normal vectors (length = 1) to the line segments
    normal_vectors = find_normal_vectors(LINEMODEL);
    
    % The Loop - REPEAT UNTIL THE PROCESS CONVERGE
    for iteration = 1:max_iterations,
        % Step 1 Translate and rotate data points
            % 1.1) Relative measurements => Sensor co-ordinates
            X_laser = DIS.*cos(ANG);
            Y_laser = DIS.*sin(ANG);

            % 1.2) Sensor co-ordinates => Robot co-ordinates
            R = [cos(Sa) -sin(Sa) Sx; sin(Sa) cos(Sa) Sy; 0 0 1]; % Rotation matrix
            RobotCoord = zeros(3, length(X_laser)); % Pre-allocate memory
            for i = 1:length(X_laser)
                RobotCoord(:, i) = R * [X_laser(i), Y_laser(i), 1]';
            end


            % 1.3) Robot co-ordinates => World co-ordinates
            % Rotate and translate using one matrix
            R = [cos(Ra) -sin(Ra) Rx; sin(Ra) cos(Ra) Ry; 0 0 1]; % Rotation matrix
            WorldCoord = zeros(3, length(X_laser)); % Pre-allocate memory
            for i = 1:length(X_laser)
                WorldCoord(:, i) = R * RobotCoord(:, i);
            end

        
        % Scatter plot the world co-ordinates to see if they are correct
        figure(1);
        scatter(WorldCoord(1,:), WorldCoord(2,:));
        % Plot each point in the LineModel and connect them to form lines
        hold on;
        for i = 1:size(LINEMODEL,1),
            plot(LINEMODEL(i,[1 3]), LINEMODEL(i,[2 4]), 'r');
        end;
        hold off;

        % Step 2 Find targets for data points
        %-> Add your code here
        
        % Step 3 Set up linear equation system, find b = (dx,dy,da)' from the LS
        %-> Add your code here
        
        b = POSE/max_iterations; % <--- You shall change this! This is only
        % for demonstation, i.e. return the same pose as sent in.
        C = 0; % Covarince matrix
        
        % Step 4 Add latest contribution to the overall congruence 
        ddx = ddx + b(1);
        ddy = ddy + b(2);
        dda = dda + b(3);
        
        % Step 5  Check if the process has converged
        %-> Add your code here
    end;

% This function calculates the normal vectors to the line segments
function normal_vectors = find_normal_vectors(LINEMODEL)
    for i = 1:size(LINEMODEL,1),
        current_line = LINEMODEL(i,:);
        current_normal = [current_line(2) - current_line(4), current_line(3) - current_line(1)];    % Normal vector to the line segment
        normal_vectors(i,:) = current_normal/norm(current_normal);  % Divide by norm to get unit vector (length = 1)
    end;