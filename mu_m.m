function [x, P] = mu_m(x, P, mag, m0, Rm)
    % mg_m update the prediciton using magnetometer measurement

    % extract elements from states
    q = x(1:4);

    % 1. prediction
    R = Qq(q);           % rotation matrix
    y_pred = R' * m0;

    % 2. innovation
    delta_y = mag - y_pred;

    % 3. jacobian
    [Q0, Q1, Q2, Q3] = dQqdq(q);
    H_q = [Q0'*m0, Q1'*m0, Q2'*m0, Q3'*m0];
    H = [H_q, zeros(3,3), zeros(3,3)];

    % 4. Kalman Filter update
    S = H * P * H' + Rm;
    K = P * H' / S;
    x = x + K * delta_y;

    % 5. normalize quaternion
    q = x(1:4);
    q = mu_normalizeQ(q);
    x(1:4) = q;

    % 6. update covariance
    P = P - K*S*K';
end