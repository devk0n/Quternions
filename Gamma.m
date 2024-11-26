function g = Gamma(q, qd)
    % Proper notation for the coordinates for each body
    r1 = q(1:3, 1);
    quat1 = q(4:7, 1);
    
    r1d = qd(1:3, 1);
    omega1 = qd(4:6, 1);
    
    A1 = Arot(quat1);
    
    % Geometry
    L1 = 1.5;
    L2 = 1.5;
    
    % Constant Vectors
    s0A = [0; 0; 0];
    
    s1Am = [-L1; 0; 0];
    s1Bm = [L2; 0; 0];
    
    % Initialization of the gamma-vector
    g = zeros(3,1);
    % Spherical joint between body 1 and ground
    g(1:3,1) = A1 * skew(omega1) * skew(s1Am) * omega1;
end
