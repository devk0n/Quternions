function q = eulerToQuat(orientation)
    % Extract roll, pitch, yaw from the input vector
    % orientation(1) = roll (x-axis rotation)
    % orientation(2) = pitch (y-axis rotation)
    % orientation(3) = yaw (z-axis rotation)
    roll = orientation(1);
    pitch = orientation(2);
    yaw = orientation(3);

    % Calculate half angles
    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);
    cp = cos(pitch * 0.5);
    sp = sin(pitch * 0.5);
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);

    % Compute quaternion components
    q = zeros(4, 1);
    q(1) = cr * cp * cy + sr * sp * sy;  % w component
    q(2) = sr * cp * cy - cr * sp * sy;  % x component
    q(3) = cr * sp * cy + sr * cp * sy;  % y component
    q(4) = cr * cp * sy - sr * sp * cy;  % z component
end
