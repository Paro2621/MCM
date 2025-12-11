%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm  % An instance of GeometricModel
        J   % Jacobian
    end

    methods
        function self = kinematicModel(gm)  % Constructor
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
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
            
            fwdGeo = [self.gm.iTj(:, :, 1)];
            
            for i = 2:linkNumber
                fwdGeo(:, :, i) = fwdGeo(:, :, i-1)*self.gm.iTj(:, :, i);
            end

            bJi  = [];

            % all the other joints
            for i = 1:linkNumber
                currentFrame = fwdGeo(:, :, i);

                k_i = currentFrame(1:3, 3);
                r_i = fwdGeo(1:3, 4, end) - currentFrame(1:3, 4);

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
            self.J = self.getJacobianOfLinkWrtBase(self.gm.jointNumber);
        end
    end
end

