function v_rot = quatRotate(q, v)
    % Function to rotate vector v using quaternion q
    % Quaternion multiplication: q * [0; v] * q'
    
    % Convert vector to quaternion form (0, v)
    v_quat = [0; v];
    
    % Quaternion conjugate
    q_conj = quatConjugate(q);
    
    % Perform quaternion multiplication q * v_quat * q_conj
    v_rot_quat = quatMultiply(quatMultiply(q, v_quat), q_conj);
    
    % Extract the vector part from the resulting quaternion
    v_rot = v_rot_quat(2:4);
end