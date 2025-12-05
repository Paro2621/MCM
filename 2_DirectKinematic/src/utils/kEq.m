function a = kEq(X, Y) % kinda Equal
    toll = 1e-6; 
    a = false;
    if abs(X-Y)<toll
        a = true;
    end
end