function plotFrame(T, name, c)
    if nargin == 2
        c = ["r" "g" "b"];
    end

    if size(T, 1) == 4  % homogeneous transformation matrix
        R = T(1:3, 1:3);
        p = T(1:3, 4);
    else                % rotation matrix
        R = T;
        p = [0; 0; 0];
    end

    for i = 1:3
        quiver3(p(1),   p(2),   p(3), ...
                R(1,i), R(2,i), R(3,i), ... 
                c(i), 'LineWidth', 1.5)
        text(p(1), p(2), p(3)-0.1, name)
        hold on
    end
end