function Phi = Phi(q, t)
    % Geometry
    r1   = q(  1:3, 1);
    phi1 = q(  4:7, 1); % Quaternion representing orientation
    r2   = q( 8:10, 1);
    phi2 = q(11:14, 1);

    % Geometry
    L1 = 1.5;
    L2 = 1.5;
    L3 = 2.0;
    
    % Constant Vectors
    s0A  = [  0; 0; 0];
    s1Am = [-L1; 0; 0];
    s1Bm = [ L2; 0; 0];
    s2Bm = [-L3; 0; 0];

    % Normalize the quaternions to avoid any numerical issues
    phi1 = phi1 / norm(phi1);
    phi2 = phi2 / norm(phi2);

    % Rotate s1Am using quaternion phi1
    s1A_rot = quatRotate(phi1, s1Am);
    s1B_rot = quatRotate(phi1, s1Bm);
    s2B_rot = quatRotate(phi2, s2Bm);

    % Preallocate the Phi vector
    Phi = zeros(6, 1);

    % Spherical joint constraint at point A
    Phi(1:3, 1) = r1 + s1A_rot - s0A;
    % Phi(4:6, 1) = r2 + s2B_rot - (r1 + s1B_rot);

    % Rotational driver for quaternion around the x-axis, y-axis, and z-axis
    target_angle_y = 0.1 * t;
    target_angle_x = 0.2 * t;
    target_angle_z = 0.5 * t;
    
    target_quat_y = angleAxisToQuat(target_angle_y, [0; 1; 0]);
    target_quat_x = angleAxisToQuat(target_angle_x, [1; 0; 0]);
    target_quat_z = angleAxisToQuat(target_angle_z, [0; 0; 1]);
    
    % Combine the target quaternions
    target_quat = quatMultiply(quatMultiply(target_quat_y, target_quat_x), target_quat_z);
    
    % Calculate the difference quaternion (error quaternion)
    error_quat = quatMultiply(quatConjugate(target_quat), phi1);
    
    % Kinematic constraint for the rotation around the y-axis and x-axis
    % Use the vector part of the quaternion (x, y, z)
    Phi(4:6, 1) = error_quat(2:4);

end