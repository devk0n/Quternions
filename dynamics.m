function [qd, reacFor, qdd] = dynamics(q)
    % External forces
    g = -10;
    % Mass properties
    m1 = 10; % Scalar mass for translation
    J_G1 = diag([1.0, 1.0, 1.0]); % Diagonal inertia tensor for simplicity

    % Mass matrix for 3D body
    M = blkdiag(diag([m1, m1, m1]), J_G1);

    % Gravity force
    Fgrav = [0.5; 0.1; 1; 0; 0; 0]; % Extend to 6x1 for the full body force vector

    % Compute Jacobian and Gamma
    J = Jacobian(q);
    G = Gamma(q(1:6), q(7:12));

    % Combined external forces
    Fext = Fgrav;
    RH = [Fext; G];

    % Ensure the dimensions are consistent
    num_constraints = 3;  % Number of constraints (based on Gamma vector size)
    num_gen_coords = size(M, 1);  % Total number of generalized coordinates

    % Adjust J to match the number of constraints and generalized coordinates
    J_ext = [J, zeros(num_constraints, num_gen_coords - size(J, 2))];

    Coef = [M, -J_ext'; J_ext, zeros(num_constraints)];

    % Solving the equations of motion and kinematic constraints
    x = Coef\RH;
    qdd = x(1:num_gen_coords,1);

    % Derivative of the state vector
    qd = [q(7:12); qdd];
    
    % Reaction forces
    lambda = x(num_gen_coords+1:end,1);
    reacFor = J' * lambda;
end
