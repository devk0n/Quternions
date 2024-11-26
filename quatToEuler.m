function orientation = quatToEuler(q)
    % Extract the values from the quaternion
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);

    % Compute Euler angles
    % Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);

    % Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x);
    if abs(sinp) >= 1
        pitch = sign(sinp) * pi / 2;  % Use 90 degrees if out of range
    else
        pitch = asin(sinp);
    end

    % Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    % Combine into a single vector
    orientation = [roll; pitch; yaw];
end