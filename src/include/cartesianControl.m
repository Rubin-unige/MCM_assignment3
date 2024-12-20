%% Kinematic Model Class - GRAAL Lab
classdef cartesianControl < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        k_a
        k_l
    end

    methods
        % Constructor to initialize the geomModel property
        function self = cartesianControl(gm,angular_gain,linear_gain)
            if nargin > 2
                self.gm = gm;
                self.k_a = angular_gain;
                self.k_l = linear_gain;
            else
                error('Not enough input arguments (cartesianControl)')
            end
        end
        
        function [x_dot, e_linear, e_angular] = getCartesianReference(self, bTg)
            %% getCartesianReference function
            % Inputs:
            % bTg : Goal frame (4x4 transformation matrix)
            % Outputs:
            % x_dot : Cartesian reference velocities (6x1 vector)
            
            % Get current tool frame w.r.t base frame
            bTt = self.gm.getToolTransformWrtBase();
            
            % Extract positions
            bOt = bTt(1:3, 4); % Position of tool frame
            bOg = bTg(1:3, 4); % Position of goal frame
            
            % Compute position error
            e_linear = bOg - bOt;
            
            % Extract rotation matrices
            bRt = bTt(1:3, 1:3); % Rotation of tool frame
            bRg = bTg(1:3, 1:3); % Rotation of goal frame
            
            % Compute relative rotation matrix from tool to goal
            tRg = bRt' * bRg;
            
            % Calculate angular error using RotToAngleAxis
            [h, theta] = RotToAngleAxis(tRg); % Use the provided function
            e_angular = bRt*h*theta; % Angular error as a 3x1 vector
            
            % Apply proportional control to compute reference velocities
            v_linear = self.k_l * e_linear;  % Linear velocity
            v_angular = self.k_a * e_angular; % Angular velocity
            
            % Combine into 6x1 Cartesian velocity vector
            x_dot = [v_angular; v_linear];
            disp(x_dot);
        end

    end
end

