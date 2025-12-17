function [iTj_0] = BuildTree()
    % This function should build the tree of frames for the chosen manipulator.
    % Inputs: 'None'
    % Outputs: The tree of frames.

    % iTj_0 corresponds to the trasformation from the frame <i> to <i'> which
    % for q = 0 is equal to the trasformation from <i> to <i+1>
    % (see notes)

    % iTj_0 is a 3-dimensional matlab matrix, suitable for defining tree of
    % frames. iTj_0 should represent the transformation matrix between the i-th and j-th
    % frames. iTj_0(row,col,joint_idx)

    i = [1 0 0]';
    j = [0 1 0]';
    k = [0 0 1]';

    iTj_0(:, :, 1) = tFactory([i j k], k.*0.105);   % <1>
    iTj_0(:, :, 2) = tFactory([k i j], k.*0.110);   % <2>
    iTj_0(:, :, 3) = tFactory([j k i], i.*0.100);   % <3>
    iTj_0(:, :, 4) = tFactory([k i j], k.*0.325);   % <4>
    iTj_0(:, :, 5) = tFactory([j k i], i.*0.095);   % <5>
    iTj_0(:, :, 6) = tFactory([i j k], k.*0.095);   % <6>
    iTj_0(:, :, 7) = tFactory([i j k], k.*0.355);   % <7> = <EE>

    % BUILDTREE - DH standard
    % iTj_0(:, :, 1) = tFactory([i j k], k.*0.105);   % <1>
    % iTj_0(:, :, 2) = tFactory([i -k j], k.*0.110);  % <2>
    % iTj_0(:, :, 3) = tFactory([i k -j], -j.*0.100); % <3>
    % iTj_0(:, :, 4) = tFactory([i -k j], k.*0.325);  % <4>
    % iTj_0(:, :, 5) = tFactory([i k -j], -j.*0.095); % <5>
    % iTj_0(:, :, 6) = tFactory([i j k], k.*0.095);   % <6>
    % iTj_0(:, :, 7) = tFactory([i j k], k.*0.355);   % <7> = <EE>
end

