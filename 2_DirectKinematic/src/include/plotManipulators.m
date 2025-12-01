%% Plot Manipulators Class - GRAAL Lab - DO NOT MODIFY!!
classdef plotManipulators < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        cmap
        color
        cindex
        csize
        show_simulation
    end

    methods
        function self = plotManipulators(show_simulation)
            self.show_simulation = show_simulation;
        end

        % Constructor to initialize the geomModel property
        function initMotionPlot(self, t) 
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
        end

        function plotIter(self, bTi)
            bri(:,1) = [0; 0; 0];
        
            for j = 1:size(bTi,3)
                bri(:,j+1) = bTi(1:3,4,j);
            end

            for j = 1:size(bri,2)
                plot3(bri(1,j), bri(2,j), bri(3,j),'bo')
            end
        
            self.color = self.cmap(mod(self.cindex,self.csize)+1,:);
            self.cindex = self.cindex + 1;
        
            line(bri(1,:), bri(2,:), bri(3,:), 'LineWidth', 1.5, 'Color', self.color)

            if self.show_simulation == true
                drawnow
            end

        end

        function plotFinalConfig(self, bTi)
            bri(:,1) = [0; 0; 0];
        
            for j = 1:size(bTi,3)
                bri(:,j+1) = bTi(1:3,4,j);
            end

            cmap = colormap("parula");
            %%Plot the final configuration of the robot
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
            cindex = 1;
            for j = 1:size(bri,2)
                plot3(bri(1,j), bri(2,j), bri(3,j),'bo')
                
            end
            
            color = cmap(mod(cindex,4)+1,:);
            cindex = cindex + 1;
            line(bri(1,:), bri(2,:), bri(3,:), 'LineWidth', 1.5, 'Color', color)
            view(gca(),[90 0]);
        end
    end
end

