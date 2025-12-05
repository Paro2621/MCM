function T_i = plotDH(DHp, T_EE , opt)
    if nargin == 2
        opt = T_EE;
        T_EE = eye(4);
    end

    plotFrame(eye(4), '<0>');
    hold on;
    axis equal;
    
    if (opt == "std")
        for i = 1:size(DHp, 1)
            T_i = DH_tFactory(DHp, i);
            plotFrame(T_i, ['<' num2str(i) '>']);
        end
        T_i = T_i*T_EE;
        plotFrame(T_i, '<EE>');
    elseif (opt == "mod")
        for i = 1:size(DHp, 1)
            T_i = mDH_tFactory(DHp, i);
            plotFrame(T_i, ['<' num2str(i) '>']);
        end
        T_i = T_i*T_EE;
        plotFrame(T_i, '<EE>');
    else
        print("plotDH: DH convention non defined")
    end
end