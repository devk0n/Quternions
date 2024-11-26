function J = Jacobian(q)
    % Proper notation for the coordinates for each body
    r1 = q(1:3, 1);
    quat1 = q(4:7, 1);
    
    A1 = Arot(quat1);
    
    % Geometry
    L1 = 1.5;
    
    % Constant Vectors
    s0A = [0; 0; 0];
    
    s1Am = [-L1; 0; 0];
    
    % Initialization of the Jacobian matrix
    I3 = eye(3);
    
    % Compute the skew-symmetric matrix for s1Am
    skew_s1Am = skew(s1Am);
    
    % Compute the quaternion derivative matrix
    quat_deriv = quat_derivative_matrix(quat1, s1Am);
    
    % Spherical joint between body 1 and ground A
    J = zeros(3, 7); % Initialize Jacobian matrix for 3 constraints and 7 variables
    J(1:3, 1:3) = I3;
    J(1:3, 4:7) = -quat_deriv;
end