function R = YPRToRot(psi, theta, phi)

    % Calculate individual rotation matrices
    Rz = [cos(psi), -sin(psi), 0;
          sin(psi), cos(psi), 0;
          0, 0, 1];

    Ry = [cos(theta), 0, sin(theta);
          0, 1, 0;
          -sin(theta), 0, cos(theta)];

    Rx = [1, 0, 0;
          0, cos(phi), -sin(phi);
          0, sin(phi), cos(phi)];
    
    % Combine the rotation matrices: R = Rz * Ry * Rx
    R = Rz * Ry * Rx;
    
end
