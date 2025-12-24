clc;
clear;
close all;

signal_load1 = load('Signal1.mat');    
noise_load1 = load('Noise1.mat');
x1 = signal_load1.y_new_1;
v2_1 = noise_load1.v2;

signal_load2 = load('Signal2.mat');    
noise_load2 = load('Noise2.mat');
x2 = signal_load2.y_new_2;
v2_2 = noise_load2.w2;

p = 4;
p = p+1;
mu = 0.01;

% Steepest descent Adaptive filter
function [y, e, w, mse, mu] = sdaf_filter(x, d, ~, p)
%   mu  - computed step size (1 / max eigenvalue of R_x)

    N = length(x);
    x = x(:); d = d(:);  % Ensure column vectors
    w = zeros(p,1);
    y = zeros(N,1);
    e = zeros(N,1);
    mse = zeros(N,1);

    x_matrix = zeros(p, N - p + 1);
    for k = p:N
        x_matrix(:, k - p + 1) = x(k:-1:k-p+1);
    end

    R_x = (x_matrix * x_matrix') / size(x_matrix, 2);

    % Compute max eigenvalue of R_x
    lambda_max = max(eig(R_x));
    mu = 1 / lambda_max;

    for n = p:N
        x_sub = zeros(p, n - p + 1);
        for k = p:n
            x_sub(:, k - p + 1) = x(k:-1:k-p+1);
        end
        R_x_local = (x_sub * x_sub') / size(x_sub,2);
        desired_vec = d(p:n);
        p_vector = (x_sub * desired_vec) / size(x_sub,2);

        % Update weights
        w = w + mu * (p_vector - R_x_local * w);

        % Compute filter output and error
        x_vec = x(n:-1:n-p+1);
        y(n) = w' * x_vec;
        e(n) = d(n) - y(n);
        mse(n) = mean(e(p:n).^2);
    end
end

% LMS algorithm
function [e, w, mse] = lms_noise_cancellation(primary, reference, p, mu)   
    N = length(primary);
    order = p;
    w = zeros(order, 1);     
    y = zeros(N, 1);  
    e = zeros(N, 1);  
    mse = zeros(N, 1); 
    
    for n = order:N
        x = reference(n:-1:n-order+1);
        
        % Filter output
        y(n) = w' * x;
        e(n) = primary(n) - y(n);
        w = w + mu * e(n) * x;
        mse(n) = mean(e(order:n).^2);
    end
end

% RLS algorithm
lambda = 0.98;
function [y, e, w, mse] = rls_anc(x, v2, p, lambda)
    delta = 0.01;
    N = length(x);          
    y = zeros(N, 1);        
    e = zeros(N, 1);        
    w = zeros(p, 1);        
    P = eye(p) / delta;      
    v2_padded = [zeros(p-1, 1); v2];
    mse = zeros(N,1);

    for n = 1:N
        u = v2_padded(n+p-1:-1:n);
        
        % Calculate the a priori error
        y(n) = w' * u;
        e(n) = x(n) - y(n);
        
        % Update the Kalman gain vector
        k = (P * u) / (lambda + u' * P * u);
        
        % Update the filter coefficients
        w = w + k * e(n);
        
        % Update the inverse correlation matrix
        P = (P - k * u' * P) / lambda;
        mse(n) = mean(e(1:n).^2);
    end
end

%% SDAF graphing
[y1, e1, w1, mse1] = sdaf_filter(x1, v2_1, mu, p);
[y2, e2, w2, mse2] = sdaf_filter(x2, v2_2, mu, p);

% Plot for SDAF
figure;
sgtitle("Filtering using SDAF method");
subplot(4,1,1);
plot(x1);
grid on; grid minor;
xlim([0 length(x1)]);
title("Noisy signal from set 1");
ylabel("Amplitude");
xlabel("Time Scale");

subplot(4,1,2);
plot(e1);
grid on; grid minor;
xlim([0 length(e1)]);
ylim([-1.5 1.7]);
title("Filtered ECG signal for Set 1");
ylabel("Amplitude");
xlabel("Time Scale");

subplot(4,1,3);
plot(x2);
grid on; grid minor;
xlim([0 length(x2)]);
title("Noisy signal from set 2");
ylabel("Amplitude");
xlabel("Time Scale");

subplot(4,1,4);
plot(e2);
grid on; grid minor;
xlim([0 length(e2)]);
ylim([-1.5 1.7]);
title("Filtered ECG signal for Set 2");
ylabel("Amplitude");
xlabel("Time Scale");

figure;
subplot(2,1,1);
plot(mse1);
grid on;grid minor;
subplot(2,1,2);
plot(mse2);
grid on; grid minor;

mse_val_1 = (mean(e1.^2));
mse_val_2 = (mean(e2.^2));

%% LMS graphing
[e3, w3, mse3] = lms_noise_cancellation(x1, v2_1, p, mu);
[e4, w4, mse4] = lms_noise_cancellation(x2, v2_2, p, mu);

% Plot for LMS
figure;
sgtitle("Filtering using LMS method");
subplot(4,1,1);
plot(x1);
grid on; grid minor;
% xlim([0 length(x1)]);
title("Noisy signal from set 1");
ylabel("Amplitude");
xlabel("Time Scale");

subplot(4,1,2);
plot(e3);
grid on; grid minor;
xlim([0 length(e3)]);
% ylim([-1.5 1.7]);
title("Filtered ECG signal for Set 1");
ylabel("Amplitude");
xlabel("Time Scale");

subplot(4,1,3);
plot(x2);
grid on; grid minor;
xlim([0 length(x2)]);
title("Noisy signal from set 2");
ylabel("Amplitude");
xlabel("Time Scale");

subplot(4,1,4);
plot(e4);
grid on; grid minor;
xlim([0 length(e4)]);
ylim([-1.5 1.7]);
title("Filtered ECG signal for Set 2");
ylabel("Amplitude");
xlabel("Time Scale");

figure;
subplot(2,1,1);
plot(mse3);
grid on;grid minor;
subplot(2,1,2);
plot(mse4);
grid on; grid minor;

mse_val_3 = (mean(e3.^2));
mse_val_4 = (mean(e4.^2));
%% RLS graphing
[y5, e5, w5, mse5] = rls_anc(x1, v2_1, p, lambda);
[y6, e6, w6, mse6] = rls_anc(x2, v2_2, p, lambda);

% Plot for RLS
figure;
sgtitle("Filtering using RLS method");
subplot(4,1,1);
plot(x1);
grid on; grid minor;
xlim([0 length(x1)]);
title("Noisy signal from set 1");
ylabel("Amplitude");
xlabel("Time Scale");

subplot(4,1,2);
plot(e5);
grid on; grid minor;
xlim([0 length(e5)]);
ylim([-1.5 1.7]);
title("Filtered ECG signal for Set 1");
ylabel("Amplitude");
xlabel("Time Scale");

subplot(4,1,3);
plot(x2);
grid on; grid minor;
xlim([0 length(x2)]);
title("Noisy signal from set 2");
ylabel("Amplitude");
xlabel("Time Scale");

subplot(4,1,4);
plot(e6);
grid on; grid minor;
xlim([0 length(e6)]);
ylim([-1.5 1.7]);
title("Filtered ECG signal for Set 2");
ylabel("Amplitude");
xlabel("Time Scale");

figure;
subplot(2,1,1);
plot(mse5);
grid on;grid minor;
subplot(2,1,2);
plot(mse6);
grid on; grid minor;

mse_val_5 = (mean(e5.^2));
mse_val_6 = (mean(e6.^2));
