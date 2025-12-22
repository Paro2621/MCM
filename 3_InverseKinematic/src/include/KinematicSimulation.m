%% Kinematic Simulation function
function q = KinematicSimulation(q, q_dot, dt, q_min, q_max)
    % Inputs
    % - q current robot configuration
    % - q_dot joints velocity
    % - dt time quantum
    % - q_min lower joints bound
    % - q_max upper joints bound
    %
    % Outputs
    % - q new joint configuration

    q = q + q_dot.*dt;
    
    for i = 1:length(q)
        if q(i) > q_max(i)
            q(i) = q_max(i);
        elseif q(i) < q_min(i)
            q(i) = q_min(i);
        end
    end
end