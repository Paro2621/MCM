function R = AngleAxisToRot(h,theta)
    % The fuction implement the Rodrigues Formula
    % Input: 
    % h is the axis of rotation
    % theta is the angle of rotation (rad)
    % Output:
    % R rotation matrix
    
    R = eye(3) + sin(theta)*skew(h) + (1-cos(theta))*skew(h)*skew(h); 
end
