function q_new = KinematicSimulation(q, q_dot, ts, q_min, q_max)
    %% Kinematic Simulation function
    %
    % Inputs
    % - q current robot configuration
    % - q_dot joints velocity
    % - ts sample time
    % - q_min lower joints bound
    % - q_max upper joints bound
    %
    % Outputs
    % - q new joint configuration
        % Compute new joint configuration based on velocity and sample time
        q_new = q + q_dot * ts;
        
        % Apply joint limits (clamping to within q_min and q_max)
        q_new = max(min(q_new, q_max), q_min);
        
end
