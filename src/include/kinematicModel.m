%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end

        function updateJacobian(self)
            %% Update Jacobian function
            % The function updates the Jacobian for the tool frame
        
            % Extract end-effector position from the last transformation
            bT7 = self.gm.getTransformWrtBase(self.gm.jointNumber); 
            bRe = bT7(1:3, 1:3);  % Rotation part
            bre = bT7(1:3, 4); % position vector of end effector wrt base
       
            % Loop through each joint
            for i = 1:self.gm.jointNumber
                % Get the transformation from base to the current joint
                bTi = self.gm.getTransformWrtBase(i); % Transformation from base to joint i
        
                % Extract joint axis and position vector from the transformation matrix
                bzi = bTi(1:3, 3); % z-axis of joint i in base frame
                bri = bTi(1:3, 4); % position vector of joint i in the base frame
                
                % Compute Jacobian based on joint type
                if self.gm.jointType(i) == 0  % Revolute joint
                    % Linear velocity
                    iJe_v = cross(bzi, (bre - bri)); 
                    
                    % Angular velocity
                    iJe_omega = bzi; 
                    
                elseif self.gm.jointType(i) == 1  % Prismatic joint
                    % Linear velocity
                    iJe_v = bzi;  
                    
                    % No angular velocity
                    iJe_omega = [0; 0; 0];  
                    
                else
                    error("Invalid joint type: must be 0 (revolute) or 1 (prismatic)");
                end
                
                % Assign to Jacobian matrix (base frame)
                self.J(1:3, i) = iJe_omega;       % Angular velocity part of the Jacobian
                self.J(4:6, i) = iJe_v;           % Linear velocity part of the Jacobian
            end
           
            % Compute the Tool Jacobian Jt (rigid body Jacobian)
            % The tool transformation relative to the end-effector
            ert = self.gm.eTt(1:3, 4);  % Position of the tool relative to the end-effector
            skew_bRe_ert = skew(bRe * ert);  % Compute the skew-symmetric matrix of (bRe * ert)
            
            % Tool Jacobian Jt
            eJt = [eye(3), zeros(3, 3); -skew_bRe_ert, eye(3)];
            
            % Compute the base-to-tool Jacobian
            bJt = eJt * self.J; % Final base-to-tool Jacobian
            self.J = bJt; 

        end

    end
end

% Skew-Symmetric Function
function S = skew(v)
    S = [  0     -v(3)   v(2);
         v(3)     0     -v(1);
        -v(2)    v(1)     0   ];
end

