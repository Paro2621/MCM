function YPR = RotToYPR(R)
    % Given a rotation matrix the function outputs the relative euler angles
    % usign the convention YPR
    % Check that R is a valid rotation matrix using IsRotationMatrix().

    if ~isRot(R)
        error("R is not a valid rotation matrix");
    end
    
    theta = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));

    if kEq(cos(theta), 0)
        % Gimbal lock: θ ≈ ±π/2
        disp("WARNING: gimbal lock detected")

        psi   = atan2(-R(1,2), R(2,2));  % arbitrary but consistent
        phi   = 0;
        
    else
        psi   = atan2(R(2,1), R(1,1));
        phi   = atan2(R(3,2), R(3,3));
        
    end

    YPR = [psi; theta; phi];
end

