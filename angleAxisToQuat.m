function quat = angleAxisToQuat(angle, axis)
    axis = axis / norm(axis); % Ensure the axis is a unit vector
    half_angle = angle / 2;
    sin_half_angle = sin(half_angle);
    quat = [
        cos(half_angle);
        axis(1) * sin_half_angle;
        axis(2) * sin_half_angle;
        axis(3) * sin_half_angle
    ];
end