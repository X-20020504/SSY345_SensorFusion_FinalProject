clear;

load('data/data_dynamic_31052025_0944_task_5_c.mat');


t0 = [];  % Initial time (initialize on first data received)
nx = 4;   % Assuming that you use q as state variable.
% Add your filter settings here.

% Current filter state.
q0 = [1; 0; 0; 0];
% x = [q0; zeros(3,1); zeros(3,1)];
x = q0;
% P = diag([0.1*ones(1,4), 0.01*ones(1,3), 0.01*ones(1,3)]);
P = diag(0.1*ones(1,4));

% Saved filter states.
xhat = struct('t', zeros(1, 0),...
              'x', zeros(nx, 0),...
              'P', zeros(nx, nx, 0));

DataLength = size(meas.gyr, 2);

prev_t = meas.t(1);

mag_q_measurment = [];
mag_q_t = [];

acc_q_measurment = [];
acc_q_t = [];

delta_ys = [];
delta_ys_t = [];
acc_ys = [];

mag_preds = [];
mag_t = [];
mag_ks = [];

% mag outlier params
L = 50;
alpha = 0.01;

for i = 1:DataLength
    if isnan(meas.gyr(1, i))  % No new data received
        continue;        % Skips the rest of the look
    end

    t = meas.t(i);
    if isempty(t0)  % Initialize t0
      t0 = t;
    end

    % mag = data(1, 8:10)';
    % if ~any(isnan(mag))  % Mag measurements are available.
    %     % Do something
    % end

    % gyro
    gyro_k = meas.gyr(:, i);
    if ~any(isnan(gyro_k))  % Gyro measurements are available.
    
        t = meas.t(i);
        T = t - prev_t;
        prev_t = t;
    
        Rw = 1e-5*[0.1231, 0.1014, -0.0020;
                   0.1014, 0.3475, -0.0089;
                   -0.0020, -0.0089, 0.0170];

        [x, P] = tu_qw4(x, P, gyro_k, T, Rw);
    end

    % acc
    acc_k = meas.acc(:, i);
    g0 = [0.0473; 0.0809; 9.8743];
    if ~any(isnan(acc_k))
    % if ~any(isnan(acc_k)) && ~accOutlier(acc_k, g0, 0.1)

        Ra = 1e-3*[0.0575, -0.0075, 0.0324;
                   -0.0075, 0.1298, -0.0651;
                   0.0324, -0.0651, 0.6122];
        
        % [x, P] = mu_g4(x, P, acc_k, Ra, g0);

    end

    % mag
    mag_k = meas.mag(:, i);
    L = (1-alpha)*L + alpha*vecnorm(mag_k, 2, 1);
    if ~any(isnan(mag_k))
    % if ~any(isnan(mag_k)) && magOutlier(mag_k, L, 5.0)
        
        mx = 1.0710;
        my = 18.8779;
        mz = -45.0483;
        m0 = [0; sqrt(mx^2+my^2); mz];

        Rm = [0.9197, 0.1279, -0.3767;
              0.1279, 0.5496,  0.0331;
             -0.3767, 0.0331,  1.1012];
        % [x, P] = mu_m4(x, P, mag_k, m0, Rm);

    end
    
    xhat.x(:, end+1) = x;
    xhat.P(:, :, end+1) = P;
    xhat.t(end+1) = t - t0;
end


figure;

taxis = linspace(1, DataLength/100, DataLength);

for i = 1:4
    subplot(4, 1, i);
    plot(meas.t, meas.orient(i, :)); hold on;
    plot(xhat.t, xhat.x(i, :)); hold on;
end

figure;
euler_orient = quat2euler(meas.orient);
euler_est = quat2euler(xhat.x);


figure;
titles = {'Roll ($\Phi$)', 'Pitch ($\theta)$', 'Yaw ($\Psi$)'};
for i = 1:3
    subplot(3, 1, i);
    plot(meas.t, euler_orient(i, :), 'LineWidth', 1.2); hold on;
    plot(xhat.t, euler_est(i, :), 'LineWidth', 1.2); hold on;

    % if i == 2
    %     ylim([-0.3, 0.3]);
    % end

    title(titles{i}, 'Interpreter', 'latex');
    xlabel('Time (s)');
    ylabel('Angle (radians)');
    legend('Benchmark', 'Filter Estimation');
    grid on;
end
sgtitle('Euler Angles Comparison: Estimated vs. Ground Truth');



% figure;
% acc_mag = vecnorm(meas.acc, 2, 1);
% plot(acc_mag);

% figure;
% 
% for i =1:3
%     subplot(3, 1, i);
%     plot(delta_ys_t, delta_ys(i, :)); hold on;
%     plot(delta_ys_t, acc_ys(i, :)); hold on;

    % plot(mag_t, mag_ks(i,:)); hold on;
    % plot(mag_t, mag_preds(i,:)); hold on;


    % plot(delta_ys_t, delta_ys(i, :)-acc_ys(i, :)); hold on;

% end

% row_var = var(delta_ys-acc_ys, 0, 2)