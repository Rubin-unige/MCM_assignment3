%% Geometric Model Class - GRAAL Lab
classdef geometricModel < handle
    % iTj_0 is an object containing the trasformations from the frame <i> to <i'> which
    % for q = 0 is equal to the trasformation from <i> to <i+1> = >j>
    % (see notes)
    % jointType is a vector containing the type of the i-th joint (0 rotation, 1 prismatic)
    % jointNumber is a int and correspond to the number of joints
    % q is a given configuration of the joints
    % iTj is  vector of matrices containing the transformation matrices from link i to link j for the input q.
    % The size of iTj is equal to (4,4,numberOfLinks)
    % eTt (OPTIONAL) add a tool to the model rigid attached to the
    % end-effector
    properties
        iTj_0
        jointType
        jointNumber
        iTj
        q
        eTt
    end

    methods
        % Constructor to initialize the geomModel property
        function self = geometricModel(iTj_0,jointType,eTt)
            if nargin > 1
                 if ~exist('eTt','var')
                     % third parameter does not exist, so default it to something
                      eTt = eye(4);
                 end
                self.iTj_0 = iTj_0;
                self.iTj = iTj_0;
                self.jointType = jointType;
                self.jointNumber = length(jointType);
                self.q = zeros(self.jointNumber,1);
                self.eTt = eTt;
            else
                error('Not enough input arguments (iTj_0) (jointType)')
            end
        end

        function updateDirectGeometry(self, q)
            %% GetDirectGeometryFunction
           % Iterate through each joint
            for i = 1:self.jointNumber
                % Extract the current transformation matrix for this joint from iTj_0
                iTj_current = self.iTj_0(:,:,i); 

                if self.jointType(i) == 0  % If the joint is rotational  
                    R_zero_state = iTj_current(1:3, 1:3);  % extract the rotational matrix from transformation
        
                    % Create new rotation matrix for rotaion around z
                    theta = q(i);
                    % rotation around z
                    R_z_qi = [cos(theta), -sin(theta), 0;
                           sin(theta), cos(theta), 0;
                           0, 0, 1];
        
                    % Multiply the zero state rotaion matrix extracted earlier
                    R_actual = R_zero_state * R_z_qi;
        
                    % Update the transformation matrix with the new rotation
                    iTj_current(1:3, 1:3) = R_actual;
        
                elseif self.jointType(i) == 1  % If the joint is prismatic
                    % Extract the translation vector from iTj_0, rotation is identity
                    r_zero_state = iTj_current(1:3, 4);  % 3x1 translation vector
        
                    % For a prismatic joint, the displacement is along the z-axis (can be different based on axis)
                    d_trans = q(i);  
        
                    % Update the translation for the prismatic joint
                    r_updated = r_zero_state + [0; 0; d_trans];  
        
                    % Update the transformation matrix with the new translation
                    iTj_current(1:3, 4) = r_updated;
                end
                
                % Update the iTj matrix for this joint
                self.iTj(:,:,i) = iTj_current;
            end
        end

        function [bTk] = getTransformWrtBase(self,k)
            %% GetTransformatioWrtBase function
            % Inputs :
            % k: the idx for which computing the transformation matrix
            % outputs
            % bTk : transformation matrix from the manipulator base to the k-th joint in
            % the configuration identified by iTj.
                        % base frame identity transformation
            bTk = eye(4);

            for i = 1:k % iterate till desired k , for instance to 7 
                bTk = bTk * self.iTj(:,:,i);
            end

        end

        function [bTt] = getToolTransformWrtBase(self)
            %% getToolTransformWrtBase function
            % Inputs :
            % None 
            % bTt : transformation matrix from the manipulator base to the
            % tool

            % Get the transformation from the base to the end-effector
            bTe = self.getTransformWrtBase(self.jointNumber); % Assuming the end-effector is the last joint
        
            % Multiply it by the tool transformation (eTt)
            bTt = bTe * self.eTt;

        end
    end
end


