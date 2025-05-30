function [x, P] = mu_g(x, P, yacc, Ra, g0)
    % mg_g update the prediciton using measurement
    % Inputs:
    %   x    - predicted states
    %   P    - predicted covariance
    %   yacc - acc measurement
    %   Ra   - acc covariance
    %   g0   - gravity
    %
    % Outputs:
    %   x    - posterior states
    %   P    - posterior covariances

    % extract elements from states
    q = x(1:4);
    b_acc = x(8:10);

    % 1. prediction
    R = Qq(q);           % rotation matrix
    y_pred = R' * g0 + b_acc;

    % 2. innovation
    delta_y = yacc - y_pred;

    % 3. jacobian
    [Q0, Q1, Q2, Q3] = dQqdq(q);
    H_q = [Q0'*g0, Q1'*g0, Q2'*g0, Q3'*g0];
    H = [H_q, zeros(3,3), eye(3)];

    % 4. Kalman Filter update
    S = H * P * H' + Ra;
    K = P * H' / S;
    x = x + K * delta_y;

    % 5. normalize quaternion
    q = x(1:4);
    q = mu_normalizeQ(q);
    x(1:4) = q;

    % 6. update covariance
    P = P - K*S*K';
end