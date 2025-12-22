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
        function x_dot = getCartesianReference(self,bTg)
            % getCartesianReference function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control
            
            bTt = self.gm.getToolTransformWrtBase();
            tTg = inv(bTt) * bTg;
            [ht,theta] = RotToAngleAxis(tTg(1:3, 1:3));

            rho_tg = ht*theta;
            rt = bTt(1:3, 4);
            rg = bTg(1:3, 4);
            r_tg = rg - rt;

            b_rho_tg = bTt(1:3, 1:3)*rho_tg;
            b_r_tg = r_tg;

            x_dot = [b_rho_tg; b_r_tg];

        end
    end
end

