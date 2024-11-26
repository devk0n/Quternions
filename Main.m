clear; close all; clc;

% Geometry
L1 = 1.5;
L2 = 1.5;
L3 = 2.0;

% Constant Vectors
s0A  = [  0; 0; 0];
s1Am = [-L1; 0; 0];
s1Bm = [ L2; 0; 0];
s2Bm = [-L3; 0; 0];

roll  = 0;
pitch = 0;
yaw   = 0;

qini = eulerToQuat([roll; pitch; yaw]);

% Initial guess
q_initial = [L1; 0; 0; qini
             (L1 + L2 + L3); 0; 0; qini];

% Refine initial guess using Newton-Raphson method
tol = 1e-6; % Tolerance for convergence
relax = 1.0; % Relaxation factor
q_refined = NewtonRaphson(@Phi, q_initial, 0, tol, relax);

disp("q_initial")
disp(q_initial)
disp("q_refined")
disp(q_refined)
disp("diff")
disp(q_initial - q_refined)

% Time for the analysis
tmin = 0;
tmax = 10;
N = 5000;
t = linspace(tmin, tmax, N)';

% Initialization of the arrays
Y = zeros(N, length(q_initial));
% Position analysis using the version of the Newton Raphson solver that
% sets up the jacobian matrix itself.
tic
for i = 1:N
    q = NewtonRaphson(@Phi, q_refined, t(i, 1), tol, relax);
    Y(i, 1:length(q_initial)) = q';
end
toc

data = Y;

% Extract the data columns
x = data(:, 1);
y = data(:, 2);
z = data(:, 3);
qw = data(:, 4);
qx = data(:, 5);
qy = data(:, 6);
qz = data(:, 7);

% Plot the 3D trajectory
figure;
scatter3(0, 0, 0, 'o');
hold on
scatter3(x, y, z, 'o');

% scatter3(Y(:,8), Y(:,9), Y(:,10), 'o');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Trajectory');
grid on;
axis equal;

% Plot quaternion components over time
figure;
subplot(4, 1, 1);
plot(qw);
ylabel('qw');
title('Quaternion Components Over Time');

subplot(4, 1, 2);
plot(qx);
ylabel('qx');

subplot(4, 1, 3);
plot(qy);
ylabel('qy');

subplot(4, 1, 4);
plot(qz);
ylabel('qz');
xlabel('Time Step');

% Optional: Plot the position components over time
figure;
subplot(3, 1, 1);
plot(x);
ylabel('X');
title('Position Components Over Time');

subplot(3, 1, 2);
plot(y);
ylabel('Y');

subplot(3, 1, 3);
plot(z);
ylabel('Z');
xlabel('Time Step');