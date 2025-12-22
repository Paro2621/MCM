%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm  % An instance of GeometricModel
        J_EEwrtB   % Jacobian
        J_EEwrtEE
    end

    methods
        function self = kinematicModel(gm)  % Constructor
            if nargin > 0
                self.gm = gm;
                self.J_EEwrtB = zeros(6, self.gm.jointNumber);
                self.J_EEwrtEE = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end

        function bJi = getJacobianOfLinkWrtBase(self,linkNumber)
            %%% getJacobianOfJointWrtBase
            % This method computes the Jacobian matrix bJi of joint i wrt base.
            % Inputs:
            % i : joint indnex ;

            % The function returns:
            % bJi
            
            for i = 1:linkNumber
                fwdGeo(:, :, i) = self.gm.getTransformWrtBase(i);
            end

            bJi  = [];
            
            r_end = fwdGeo(1:3, 4, end);

            % all the other joints
            for i = 1:linkNumber
                currentFrame = fwdGeo(:, :, i); % wrt base

                k_i = currentFrame(1:3, 3);
                r_i = r_end - currentFrame(1:3, 4);

                if self.gm.jointType(i) == 0        % revolute
                    J_i = [k_i; cross(k_i, r_i)];
                    bJi = [bJi J_i];
                elseif self.gm.jointType(i) == 1    % prismatic
                    J_i = [0; 0; 0; k_i];
                    bJi = [bJi J_i];
                end
            end

            for i = linkNumber+1:self.gm.jointNumber
                J_i = [0 0 0 0 0 0]';
                bJi = [bJi J_i];
            end
        end

        function updateJacobian(self)
            %%% Update Jacobian function 
            % The function update: 
            % - J: end-effector jacobian matrix
            
            % Jacobian expressed in the Base frame
            self.J_EEwrtB = self.getJacobianOfLinkWrtBase(self.gm.jointNumber);

            % Base -> EE
            b_T_n = self.gm.getTransformWrtBase(self.gm.jointNumber);
            n_T_e = tFactory(eye(3), [0 0 0.06]');
            b_T_e = b_T_n; % * n_T_e;

            % Inverse transform (EE -> Base) for twist transformation
            e_T_b = invert(b_T_e);

            R = e_T_b(1:3, 1:3);
            p = e_T_b(1:3, 4);
            
            Ad = [R, zeros(3,3); skew(p)*R, R];

            % Jacobian expressed in the EE frame
            self.J_EEwrtEE = Ad * self.J_EEwrtB;
        end
    end
end

