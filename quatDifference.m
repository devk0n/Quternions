function diff = quatDifference(q1, q2)
    % Quaternion difference q1 - q2
    % Convert quaternions to the same frame and compute the difference
    % Ensure the output is a 4x1 vector
    q2_conj = [q2(1); -q2(2:4)]; % Conjugate of q2
    q_diff = quatMultiply(q1, q2_conj); % Difference as quaternion multiplication
    diff = q_diff(:); % Ensure column vector output
end