function q_conj = quatConjugate(q)
    % Conjugate of quaternion q
    q_conj = [q(1); -q(2:4)];
end