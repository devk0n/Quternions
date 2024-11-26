clear; close all; clc;

% Define orientation vector (roll, pitch, yaw) in radians
position = [L1; 0; 0];
orientation = [0; 0; 0];  % [60 degrees; 30 degrees; 45 degrees]

% Convert Euler angles to quaternion
q = eulerToQuat(orientation);
disp('Quaternion:');
disp(q);

% Convert the quaternion back to Euler angles
orientation_back = quatToEuler(q);
disp('Euler angles (radians):');
disp(orientation_back);

% Convert Euler angles to degrees for better readability
orientation_degrees = rad2deg(orientation_back);
disp('Euler angles (degrees):');
disp(orientation_degrees);

% Geometry
L1 = 1.5;

% Constant Vectors
s0A  = [  0; 0; 0];
s1Am = [-L1; 0; 0];

q_initial = [0; 0; 0; 1; 0; 0; 0; % Body 1 (position and orientation)
             0; 0; 0; 0; 0; 0; 0]; % Velocity

function q = eulerToQuat(orientation)
    % Extract roll, pitch, yaw from the input vector
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
    q = zeros(1, 4);
    q(1) = cr * cp * cy + sr * sp * sy;  % w
    q(2) = sr * cp * cy - cr * sp * sy;  % x
    q(3) = cr * sp * cy + sr * cp * sy;  % y
    q(4) = cr * cp * sy - sr * sp * cy;  % z
end



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

