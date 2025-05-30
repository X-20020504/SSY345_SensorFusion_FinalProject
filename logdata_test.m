clear;

load('data_29_may_task_4_with_orientation.mat');


t0 = [];  % Initial time (initialize on first data received)
nx = 10;   % Assuming that you use q as state variable.
% Add your filter settings here.

% Current filter state.
q0 = [1; 0; 0; 0];
x = [q0; zeros(3,1); zeros(3,1)];
P = diag([0.1*ones(1,4), 0.01*ones(1,3), 0.01*ones(1,3)]);

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
    
        gyro_noise = 1e-5;
        bias_omega_noise = 1e-7;
        bias_acc_noise = 1e-6;
        Rw = diag([gyro_noise*ones(1,3), bias_omega_noise*ones(1,3), bias_acc_noise*ones(1,3)]);
    
        [x, P] = tu_qw(x, P, gyro_k, T, Rw);
    end

    % acc
    acc_k = meas.acc(:, i);
    if ~any(isnan(acc_k)) && ~accOutlier(acc_k, 1.0)
    
        sigma_a = 0.05;
        Ra = diag([sigma_a^2, sigma_a^2, sigma_a^2]);
        % Ra = diag([0.1200, 0.0398, 0.1390]);
    
        g0 = [0; 0; 9.81];
    
        [x, P] = mu_g(x, P, acc_k, Ra, g0);
    end

    % mag
    mag_k = meas.mag(:, i);
    if ~any(isnan(mag_k))
        
        sigma_m = 0.4;
        Rm = diag([sigma_m^2, sigma_m^2, sigma_m^2]);
    
        mx = -5.2360;
        my = 38.0975;
        mz = -46.0552;
        m0 = [0; sqrt(mx^2+my^2); mz];


        Qq_measurement_trans = mag_k * pinv(m0);
        mag_q_measurment(:, end+1) = rotMat2quat(Qq_measurement_trans');
        mag_q_t(:, end+1) = t-t0;
    
        [x, P] = mu_m(x, P, mag_k, m0, Rm);

        q = x(1:4);
        R = Qq(q);           % rotation matrix
        mag_pred = R' * m0;
        delta_mag = mag_k - mag_pred;
        % delta_ys(:, end+1) = delta_y;
        mag_preds(:, end+1) = mag_pred;
        mag_t(:, end+1) = t - t0;
        mag_ks(:, end+1) = mag_k;


    end
    
    xhat.x(:, end+1) = x;
    xhat.P(:, :, end+1) = P;
    xhat.t(end+1) = t - t0;
end


figure;

taxis = linspace(1, DataLength/100, DataLength);

for i =1:4
    subplot(4, 1, i);
    plot(meas.t, meas.orient(i, :)); hold on;
    plot(xhat.t, xhat.x(i, :)); hold on;

    % scatter(mag_q_t, mag_q_measurment(i, :));
    % scatter(acc_q_t, acc_q_measurment(i, :), 10, 'filled');
end


% figure;
% acc_mag = vecnorm(meas.acc, 2, 1);
% plot(acc_mag);

figure;

for i =1:3
    subplot(3, 1, i);
    % plot(delta_ys_t, delta_ys(i, :)); hold on;
    % plot(delta_ys_t, acc_ys(i, :)); hold on;

    plot(mag_t, mag_ks(i,:)); hold on;
    plot(mag_t, mag_preds(i,:)); hold on;


    % plot(delta_ys_t, delta_ys(i, :)-acc_ys(i, :)); hold on;

end

% row_var = var(delta_ys-acc_ys, 0, 2)