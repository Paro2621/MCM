function out = invert(A) 
    % return the inverse of a given homogeneous transformation matrix
    R_t = A(1:3, 1:3)';
    r = A(1:3, 4);
    
    out = eye(4);
    out(1:3, 1:3) = R_t;
    out(1:3, 4) = -R_t*r;
end