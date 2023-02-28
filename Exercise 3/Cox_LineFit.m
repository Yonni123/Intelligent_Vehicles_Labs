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
function [ddx,ddy,dda,C] = Cox_LineFit(ANG, DIS, POSE, LINEMODEL, SensorPose, II)
    % Init variables
    ddx = 0; ddy = 0; dda = 0;  % The translation and rotation to be returned
    Rx = POSE(1,1); Ry = POSE(2,1); Ra = POSE(3,1);     % The initial pose (in world co-ordinates)
    Sx = SensorPose(1); Sy = SensorPose(2); Sa = SensorPose(3);   % X, Y and Theta of the sensor location (in robot co-ordinates)
    max_iterations = 100;
    no_update = 0;

    if (II == 272),
        disp('STOP HERE');
    end
    
    % Step 0 - Normal vectors (length = 1) to the line segments
    normal_vectors = find_normal_vectors(LINEMODEL);
    
    % The Loop - REPEAT UNTIL THE PROCESS CONVERGE
    for iteration = 1:max_iterations,
        % Step 1 Translate and rotate data points
            % 1.1) Relative measurements => Sensor co-ordinates
            X_laser = DIS.*cos(ANG);
            Y_laser = DIS.*sin(ANG);
            SensorCoord = [X_laser; Y_laser; ones(1, length(X_laser))]; % Coordinates relative to the sensor

            % 1.2) Sensor co-ordinates => Robot co-ordinates
            R = [cos(Sa) -sin(Sa) Sx; sin(Sa) cos(Sa) Sy; 0 0 1];
            RobotCoord = R * SensorCoord; % Coordinates relative to the robot

            % 1.3) Robot co-ordinates => World co-ordinates
            R = [cos(Ra+dda) -sin(Ra+dda) Rx+ddx; sin(Ra+dda) cos(Ra+dda) Ry+ddy; 0 0 1];
            WorldCoord = R * RobotCoord; % Coordinates relative to the world

            %plot_points_and_lineModel(WorldCoord, LINEMODEL, 6);

        % Step 2 Find targets for data points
            % 2.1) Assign each data point to the closest line segment
            TargetLine = zeros(size(WorldCoord,2),1); % The index of the line segment that each data point is closest to
            MinDist = zeros(size(WorldCoord,2),1);    % The distance from each data point to the closest line segment (used for pruning outliers)
            for i = 1:size(WorldCoord,2),   % For each data point
                Dists = zeros(size(LINEMODEL,1),1); % Distances from the data point each line segments
                for j = 1:size(LINEMODEL,1),
                    current_line = LINEMODEL(j,:);
                    current_point = WorldCoord(1:2,i)';
                    vi = current_point; % Just to be consistent with the notation in the PDF

                    ui = normal_vectors(j,:);   % Normal vector to the line segment

                    z = current_line(1:2); % Point on the line segment
                    ri = dot(ui,z); % Distance from origo to the line segment

                    % Distance from the data point to the line segment
                    yi = ri - dot(ui,vi);

                    Dists(j) = yi;
                end
                % Find the line segment with the smallest distance
                [~, min_index] = min(abs(Dists));
                TargetLine(i) = min_index;
                MinDist(i) = Dists(min_index);  % Not same as min_dist since Dists contains the actual distance, not the absolute value of it;
            end;

            % 2.2) Prune outliers
            threshold = 30;    % Threshold distance at which a data point is considered an outlier
            [WorldCoord, TargetLine, MinDist] = prune_outliers(WorldCoord, TargetLine, MinDist, threshold);
            % plot_points_and_lineModel(WorldCoord, LINEMODEL, 10);
        
        % Step 3 Set up linear equation system, find b = (dx,dy,da)' from the LS
        ui = normal_vectors(TargetLine,:);   % Normal vectors to the line segments
        ui1 = ui(:,1); ui2 = ui(:,2);   % Separate the x and y components of the normal vectors
        xi1 = ui1;
        xi2 = ui2;

        vi = WorldCoord(1:2,:)';    % Data points
        vm = [Rx; Ry];
        % Subtract every data point with the robot position (vi - vm)
        diff = vi - repmat(vm', size(vi,1), 1);
        something = [0 -1;1 0] * diff';
        xi3 = dot(ui', something);

        A = [xi1 xi2 xi3'];
        y = MinDist;    % The distance from each data point to the closest line segment
        b = inv(A'*A)*A'*y;    % <--- This is the solution to the linear equation system

        n = max(size(A));   % Number of data points
        yab = ( A*b - y )   % Residuals
        yab2 = yab.^2;      % Squared residuals
        summa = sum(yab2);  % Sum of squared residuals
        s2 = summa/(n-4);   % Variance of the residuals
        C = s2 * inv(A'*A);    % Covariance matrix
        
        % Step 4 Add latest contribution to the overall congruence 
        ddx = ddx + b(1);
        ddy = ddy + b(2);
        dda = dda + b(3);
        
        % Step 5  Check if the process has converged
        if (sqrt(b(1)^2 + b(2)^2) < 5 )&&(abs(b(3) <0.1*pi/180) ),
            break;  % stop loop
        end
        %plot_points_and_lineModel(WorldCoord, LINEMODEL, 6);
    end;

% This function calculates the normal vectors to the line segments
function normal_vectors = find_normal_vectors(LINEMODEL)
    for i = 1:size(LINEMODEL,1),
        current_line = LINEMODEL(i,:);
        current_normal = [current_line(2) - current_line(4), current_line(3) - current_line(1)];    % Normal vector to the line segment y1 - y2, x2 - x1
        normal_vectors(i,:) = current_normal/norm(current_normal);  % Divide by norm to get unit vector (length = 1)
    end;

% This function removes outliers from the data points that's a threshold distance away from the line segments
function [pruned_coordinates, new_target_lines, MinDist] = prune_outliers(coordinates, target_lines, dist_from_line, threshold)
    pruned_coordinates = [];    
    new_target_lines = [];      % Start with empty list of pruned data points and add the non-outliers to it
    MinDist = [];
    for i = 1:size(coordinates,2),
        if abs(dist_from_line(i)) < threshold,
            pruned_coordinates = [pruned_coordinates coordinates(:,i)]; % Add the data point to the list of pruned data points
            new_target_lines = [new_target_lines; target_lines(i)]; % Add the target line to the list of pruned target lines
            MinDist = [MinDist; dist_from_line(i)]; % Add the distance to the list of pruned distances
        end
    end

function plot_points_and_lineModel(points, lineModel, figureNumber)
    % New figure
    figure(figureNumber);
    hold on;
    plot(points(1,:), points(2,:), 'ro');
    for i = 1:size(lineModel,1),
        current_line = lineModel(i,:);
        plot([current_line(1) current_line(3)], [current_line(2) current_line(4)], 'b');
    end
    hold off;