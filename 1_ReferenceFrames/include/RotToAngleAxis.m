function [h,theta] = RotToAngleAxis(R)
    % Given a rotation matrix this function
    % should output the equivalent angle-axis representation values,
    % respectively 'theta' (angle), 'h' (axis)
    % Check that R is a valid rotation matrix using IsRotationMatrix()

    if isRot(R) == false
        disp("ERROR: R is not a valid rotation matrix")
    else
        theta = acos((trace(R)-1)/2);

        if kEq(theta, 0)
            h = [0; 0; 0];
        elseif kEq(theta, pi)
            h = [sqrt((R(1,1)+1)/2); ...
                 sign(R(1,2))*sqrt((R(2,2)+1)/2); ...
                 sign(R(1,3))*sqrt((R(3,3)+1)/2)];
        else
            sk_h = (R - R')/(2*sin(theta));
            h = vex(sk_h);
        end
    end
end