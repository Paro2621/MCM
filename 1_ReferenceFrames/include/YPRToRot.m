function R = YPRToRot(v, theta, phi)
    % The function compute the rotation matrix using the YPR (yaw-pitch-roll)
    % convention, given psi, theta, phi.
    % Input:
    % psi angle around z axis (yaw)
    % theta angle around y axis (pitch)
    % phi angle around x axis (roll)
    % Output:
    % R rotation matrix

    if nargin == 1
        psi = v(1);
        theta = v(2);
        phi = v(3);
    else
        psi = v;
    end

    R = Rz(psi)*Ry(theta)*Rx(phi);
end