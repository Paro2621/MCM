function ret = isRot(A)
    % return true if A is a valid rotation matrix s.t: 
    % 1. A*A' == eye(3)     --> A = eye(3)
    % 2. det(A) == 1        --> a == 1
    
    a = det(A);
    A = A*A';

    for i = 1:3
        for j = 1:3
            if kEq(A(i,j), 0)
                A(i, j) = 0;
            elseif kEq(A(i,j), 1)
                A(i, j) = 1;
            end
        end
    end

    if kEq(a, 1)
        a = 1;
    end

    % after fixing numerical instabilities
    if (all(A == eye(3))*(a == 1))
        ret = true;
    else
        ret = false;
    end
end