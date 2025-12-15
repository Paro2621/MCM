%% Plot Manipulators Class - GRAAL Lab
%% DO NOT MODIFY!!
classdef plotManipulators < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        cmap
        color
        cindex
        csize
        show_simulation
        bri
        x_dot_hist
        t_hist
    end

    methods
        function self = plotManipulators(show_simulation)
            self.show_simulation = show_simulation;
            self.x_dot_hist = [];
            self.t_hist = [];
        end

        % Constructor to initialize the geomModel property
        function initMotionPlot(self, t, bP) 
            figure
            grid on 
            hold on
            title('MOTION OF THE MANIPULATOR')
            xlabel('x')
            ylabel('y')
            zlabel('z')
            axis equal
            az = 48;
            el = 25;
            view(az,el)
            self.cindex = 1;
            self.csize = length(t);
            self.cmap = colormap(parula(self.csize));
            self.color = self.cmap(mod(self.cindex,self.csize)+1,:);
            plot3(bP(1),bP(2),bP(3),'ro')
        end

        function plotIter(self, gm, km, i, q_dot)
            for j=1:gm.jointNumber
                bTi(:,:,j) = gm.getTransformWrtBase(j); 
            end
        
            bri(:,1) = [0; 0; 0];
            % Plot joints
            for j = 1:gm.jointNumber
                bri(:,j+1) = bTi(1:3,4,j);              
            end
            bTt = gm.getToolTransformWrtBase();
            bri(:,gm.jointNumber+2) = bTt(1:3,4); 

            if (rem(i,0.1) ~= 0)
                return;
            end
    
            % Plot links
            for j = 1:gm.jointNumber+1
                plot3(bri(1,j), bri(2,j), bri(3,j),'bo')           
            end
            plot3(bri(1,gm.jointNumber+2),bri(2,gm.jointNumber+2),bri(3,gm.jointNumber+2),'go') 
        
            self.color = self.cmap(mod(self.cindex,self.csize)+1,:);
            self.cindex = self.cindex + 1;
        
            line(bri(1,:), bri(2,:), bri(3,:), 'LineWidth', 1.5, 'Color', self.color)
            if self.show_simulation == true
                drawnow
            end
            self.bri = bri;

            x_dot_actual = km.J*q_dot;

            self.x_dot_hist = [self.x_dot_hist; (x_dot_actual/norm(x_dot_actual))'];
            self.t_hist = [self.t_hist; i];

        end

        function plotFinalConfig(self, gm)
            figure
            grid on 
            hold on
            title('FINAL CONFIGURATION')
            xlabel('x')
            ylabel('y')
            zlabel('z')
            axis equal
            az = 48;
            el = 25;
            view(az,el)
            self.cindex = 1;
            for j = 1:gm.jointNumber+2
                plot3(self.bri(1,j), self.bri(2,j), self.bri(3,j),'bo')
                
            end
            
            self.color = self.cmap(mod(self.cindex,self.csize)+1,:);
            self.cindex = self.cindex + 1;
            
            bri = self.bri;
            line(bri(1,:), bri(2,:), bri(3,:), 'LineWidth', 1.5, 'Color', self.color)
            
            figure
            hold on;
            title('DIRECTION OF THE END-EFFECTOR VELOCITIES')
            plot(self.t_hist, self.x_dot_hist)
            legend('omega x', 'omega y', 'omega z', 'xdot', 'ydot', 'zdot')
        end
    end
end

