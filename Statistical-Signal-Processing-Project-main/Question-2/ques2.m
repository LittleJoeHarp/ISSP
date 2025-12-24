% Aryan Shrivastava - 2023102025

% Problem 2
clc;
clear;
close all;

%% Part A

% Initial Values provided
v_r = 10;
v_o = pi/60;
timescale = 0:1:200;
radial_dist = 1000+v_r.*timescale;
angular_dist = v_o.*timescale;

% For Trajectory (Y-axis, X-axis)
x_t = radial_dist.*cos(angular_dist);
y_t = radial_dist.*sin(angular_dist);


% Animation (aesthetics...LOL)
figure;
for i = 1:length(timescale)
    plot(x_t(1:i), y_t(1:i), 'b-', x_t(i), y_t(i), 'ro');
    title("Siva's Trajectory with Constant Radial and Angular Velocity", 'FontWeight', 'bold');
    grid on;
    grid minor;
    axis equal;
    xlabel('x_t (m)'); ylabel('y_t (m)');
    subtitle(['Time = ', num2str(timescale(i)), ' s']);
    axis([min(x_t)-100 max(x_t)+100 min(y_t)-100 max(y_t)+100]);
    grid on;
    drawnow;
end

% Markers (Aesthetics again...LOL)
hold on;
for t_mark = 0:50:200
    i = t_mark + 1; 
    plot(x_t(i), y_t(i), 'k*', 'MarkerSize', 5);
    text(x_t(i), y_t(i), sprintf('t=%ds', t_mark), 'VerticalAlignment', 'bottom');
end
hold off;

%% Part B and Part C done in report
% Part D
ranged1 = load("range1.mat");
angled1 = load("angle1.mat");
ranged1 = ranged1.range1;   
angled1 = deg2rad(angled1.angle1);

% Kalman Filter setup

% x = [r; θ; v_r; v_θ]
% F = state transition matrix
% H = Measurement matrix

delta_t = 1; 
F = [1 0 delta_t 0; 0 1 0 delta_t; 0 0 1 0; 0 0 0 1];
H = [1 0 0 0; 0 1 0 0];
% Q = diag([0.1, 0.1, 0.01, 0.01]); % Process noise
Q = zeros(4);
R = diag([2500, 16]);
x_est = [500; 0; 0; 0];
P_est = diag([1000, 10, 50, 10]);
% % 
% Comparision purpose
% P_est = diag([1e6, 1e6, 1e6, 1e6]); % High initial uncertainty
% P_est = diag([1, 1, 1, 1]); % Very low initial uncertainty

N = length(ranged1);
estimated_states = zeros(4, N);
estimated_positions = zeros(2, N);

for k = 1:length(ranged1)
    % Prediction
    x_pred = F * x_est;
    P_pred = F * P_est * F' + Q;
    
    % Update
    z = [ranged1(k); angled1(k)];
    K = P_pred * H' / (H * P_pred * H' + R);
    x_est = x_pred + K * (z - H * x_pred);
    P_est = (eye(4) - K * H) * P_pred;
    
    % Store position
    estimated_states(:, k) = x_est;
    estimated_positions(:, k) = [x_est(1)*cos(x_est(2)); x_est(1)*sin(x_est(2))];
end

% Plot result
figure;
plot(estimated_positions(1,:), estimated_positions(2,:), 'b-', 'LineWidth', 1.5);
grid on;
grid minor;
axis equal;
xlabel('x_estimated (m)'); ylabel('y_estimated (m)');
sgtitle("Estimated Trajectory by Madhuri")

hold on;
for t_mark = 0:50:200
    i = t_mark + 1; 
    plot(estimated_positions(1,i), estimated_positions(2,i), 'k*', 'MarkerSize', 5);
    text(estimated_positions(1,i), estimated_positions(2,i), sprintf('t=%ds', t_mark), 'VerticalAlignment', 'bottom');
end
hold off;

figure;
subplot(2,1,1);
plot(0:200, estimated_states(3,:));
title('Estimated Radial Velocity');
ylabel('v_r (m/s)');
grid on;grid minor;

subplot(2,1,2);
plot(0:200, estimated_states(4,:));
title('Estimated Angular Velocity');
xlabel('Time (s)'); ylabel('v_\theta (rad/s)');
grid on;grid minor;

%% Part E done in report

% Part f from here onwards
ranged2 = load("range2.mat");
angled2 = load("angle2.mat");
ranged2 = ranged2.range2;   
angled2 = deg2rad(angled2.angle2);

delta_t = 1; 
F = [1 0 delta_t 0; 0 1 0 delta_t; 0 0 1 0; 0 0 0 1];
H = [1 0 0 0; 0 1 0 0; 1 0 0 0; 0 1 0 0];
R = diag([2500, 16, 2500, 16]);
Q = zeros(4);
x_est = [500; 0; 0; 0];
P_est = diag([1000, 10, 50, 10]);
num_steps = length(ranged2); 
estimated_states2 = zeros(4, num_steps);
estimated_positions2 = zeros(2, num_steps);

for k = 1:num_steps
    % Prediction Step 
    x_pred = F * x_est;
    P_pred = F * P_est * F' + Q;
    
    % Update Step 
    z_k = [ranged1(k); angled1(k); ranged2(k); angled2(k)];
    
    % Kalman update
    K = P_pred * H' / (H * P_pred * H' + R);
    x_est = x_pred + K * (z_k - H * x_pred);
    P_est = (eye(4) - K * H) * P_pred;
    
    estimated_states2(:, k) = x_est;
    estimated_positions2(:, k) = [x_est(1)*cos(x_est(2)); x_est(1)*sin(x_est(2))];
end

% Plot result
figure;
plot(estimated_positions2(1,:), estimated_positions2(2,:), 'b-', 'LineWidth', 1.5);
grid on;
grid minor;
axis equal;
xlabel('x_ estimated (m)'); ylabel('y_ estimated (m)');
sgtitle("Estimated Trajectory by Madhuri using dual sensor")


hold on;
for t_mark = 0:50:200
    i = t_mark + 1; 
    plot(estimated_positions2(1,i), estimated_positions2(2,i), 'k*', 'MarkerSize', 5);
    text(estimated_positions2(1,i), estimated_positions2(2,i), sprintf('t=%ds', t_mark), 'VerticalAlignment', 'bottom');
end
hold off;

figure;
subplot(2,1,1);
plot(0:200, estimated_states2(3,:));
title('Estimated Radial Velocity');
ylabel('v_r (m/s)');
grid on;grid minor;

subplot(2,1,2);
plot(0:200, estimated_states2(4,:));
title('Estimated Angular Velocity');
xlabel('Time (s)'); ylabel('v_\theta (rad/s)');
grid on;grid minor;

%% Comparision

% For radial dist
error_min_single = min(radial_dist - estimated_states(1,:));  
error_min_dual = min(radial_dist - estimated_states2(1,:));
error_mean_single = sqrt(mean((radial_dist - estimated_states(1,:)).^2));
error_mean_dual = sqrt(mean((radial_dist - estimated_states2(1,:)).^2));

fprintf('Single-Sensor observations\n');
fprintf('RMSE recorded: %.2f m\n',error_mean_single);
fprintf('Minimum error recorded: %.2f\n\n',abs(error_min_single));

fprintf('Dual-Sensor observations\n');
fprintf('RMSE recorded: %.2f m\n',error_mean_dual);
fprintf('Minimum error recorded: %.2f\n\n',abs(error_min_dual));

% For radial velocity
rad_vel_1 = sqrt(mean((v_r - estimated_states(3,:)).^2));  
rad_vel_2 = sqrt(mean((v_r - estimated_states2(3,:)).^2)); 
fprintf('Single-Sensor Radial Velocity RMSE : %.2f m\n', rad_vel_1);
fprintf('Dual-Sensor Radial Velocity RMSE : %.2f m\n\n', rad_vel_2);

% For angular velocity
ang_vel_1 = sqrt(mean((v_o - estimated_states(4,:)).^2));  
ang_vel_2 = sqrt(mean((v_o - estimated_states2(4,:)).^2)); 
fprintf('Single-Sensor Radial Velocity RMSE : %.2f m\n', ang_vel_1);
fprintf('Dual-Sensor Radial Velocity RMSE : %.2f m\n\n', ang_vel_2);

x_t = x_t(:);
estimated = estimated_positions(1,:).';
abs_error = abs(x_t - estimated);
cumulative_mean_error = cumsum(abs_error) ./ (1:length(abs_error))';
estimated2 = estimated_positions2(1,:).';
abs_error2 = abs(x_t - estimated2);
cumulative_mean_error2 = cumsum(abs_error2) ./ (1:length(abs_error2))';
figure;
subplot(2,1,1);
plot(cumulative_mean_error);
xlabel('Time Step');
ylabel('Cumulative Mean Absolute Error');
title('Cumulative Mean of Absolute Error for Radial Distance for single sensor');
grid on;
grid minor;

subplot(2,1,2);
plot(cumulative_mean_error2);
xlabel('Time Step');
ylabel('Cumulative Mean Absolute Error');
title('Cumulative Mean of Absolute Error for Radial Distance for dual sensor');
grid on;
grid minor;





