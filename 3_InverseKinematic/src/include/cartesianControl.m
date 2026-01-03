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

        function [x, x_dot] = getCartesianReferenceEE(self, bTg)
            % getCartesianReference function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control
            
            bTe = self.gm.getTransformWrtBase(self.gm.jointNumber);

            eTg = bTe\bTg;
            [ht,theta] = RotToAngleAxis(eTg(1:3, 1:3));

            rho_eg = ht*theta;
            re = bTe(1:3, 4);
            rg = bTg(1:3, 4);
            r_eg = rg - re;

            b_rho_eg = bTe(1:3, 1:3)*rho_eg;
            b_r_eg = r_eg;

            x = [b_rho_eg; b_r_eg];
            x_dot = [self.k_a.*b_rho_eg; self.k_l.*b_r_eg];
        end

        function [x, x_dot] = getCartesianReferenceTool(self, bTg)
            % getCartesianReference function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control
            
            bTt = self.gm.getToolTransformWrtBase();

            tTg = bTt\bTg;

            [ht,theta] = RotToAngleAxis(tTg(1:3, 1:3));

            rho_tg = ht*theta;
            r_tg = bTg(1:3, 4) - bTt(1:3, 4);

            b_rho_tg = bTt(1:3, 1:3)*rho_tg;
            b_r_tg = r_tg;
            
            x = [b_rho_tg; b_r_tg];
            x_dot = [self.k_a.*b_rho_tg; self.k_l.*b_r_tg];
        end
    end
end
