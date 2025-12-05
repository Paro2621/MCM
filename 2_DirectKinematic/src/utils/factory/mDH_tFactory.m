function T = mDH_tFactory(DHp, n)
    T = eye(4);

    for i = n:-1:1
        d_i     = DHp(i, 1);
        theta_i = DHp(i, 2);
        r_i     = DHp(i, 3);
        alpha_i = DHp(i, 4);

        DH_T_i = ...
            tFactory(eye(3),        [r_i; 0; 0])*  ...
            tFactory(Rx(alpha_i),   [0;   0; 0])*  ... 
            tFactory(eye(3),        [0;   0; d_i])*...
            tFactory(Rz(theta_i),   [0;   0; 0]);

        T = DH_T_i*T;
    end
end
