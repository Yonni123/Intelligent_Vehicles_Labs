function TEST_SHIT(LINEMODEL)
    normal_vectors = zeros(size(LINEMODEL,1),2);  % Preallocate memory for the normal vectors
    for i = 1:size(LINEMODEL,1),
        current_line = LINEMODEL(i,:);
        current_normal = [current_line(2) - current_line(4), current_line(3) - current_line(1)];    % Normal vector to the line segment
        normal_vectors(i,:) = current_normal/norm(current_normal);  % Divide by norm to get unit vector (length = 1)
    end;

    % Plot the normal vectors and the line segments to see if they are correct with a legend
    figure;
    hold on;
    for i = 1:size(LINEMODEL,1),
        current_line = LINEMODEL(i,:);
        current_normal = normal_vectors(i,:);
        plot([current_line(1) current_line(3)], [current_line(2) current_line(4)], 'b');
        hold on;
        plot([current_line(1) current_line(1) + current_normal(1)], [current_line(2) current_line(2) + current_normal(2)], 'r');
        % add a legend
        legend('Line segment', 'Normal vector');
    end;
    hold off;