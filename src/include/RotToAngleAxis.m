function [h, theta] = RotToAngleAxis(R)
    % Check if R is a 3x3 matrix
    if size(R, 1) ~= 3 || size(R, 2) ~= 3
        error('Input must be a 3x3 matrix.');
    end

    % Check if R is a proper orthogonal matrix
    if abs(det(R) - 1) > 1e-6 || norm(R' * R - eye(3)) > 1e-6
        error('Input matrix is not a valid rotation matrix.');
    end

    % Calculate the angle theta
    theta = acos((trace(R) - 1) / 2);
    
    % Calculate the rotation axis h
    if theta == 0
        h = [0; 0; 0]; % No rotation
    else
        % Calculate the skew-symmetric part
        S = (R - R') / (2 * sin(theta));
        h = vex(S); % Convert skew-symmetric matrix to vector
    end
end

function a = vex(S_a)
    % Input: skew matrix S_a (3x3)
    % Output: the original a vector (3x1)
    a = [S_a(3, 2); S_a(1, 3); S_a(2, 1)];
end
