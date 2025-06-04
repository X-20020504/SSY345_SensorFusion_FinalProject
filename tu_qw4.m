function [x, P] = tu_qw4(x, P, omega, T, Rw)
%   time update function
%   Inputs:
%     x     - current state [q] (4×1)
%     P     - current covarience matrix (4×4)
%     omega - measured angular speed from gyro (3×1)
%     T     - sampling time
%     Rw    - process noise covariance matrix (3×3)
%
%   Outputs:
%     x     - predicted states
%     P     - predicted covarience matrix

    % state_vector
    q = x;              % quaternion
    
    % 1. state prediction --------------------------------------------------------------
    % quaternion prediction
    F_omega = eye(4) + 0.5*T*Somega(omega);
    x_pred = F_omega * q;
    x_pred = mu_normalizeQ(x_pred);
    
    % 2. covariance prediction ------------------------------------------------------------
    Gq = Gqfunc(q, T);
  
    % matrix A (10×10)
    A = F_omega;      % ∂q/∂q (4×4)
    
    % matrix B (10×9)
    B = Gq;           % ∂q/∂v_omega (4×3)
    
    % covariance matrix: P_k = A*P_{k-1}*A' + B*Rw*B'
    P_pred = A * P * A' + B * Rw * B';
    
    % final results
    x = x_pred;
    P = P_pred;

end