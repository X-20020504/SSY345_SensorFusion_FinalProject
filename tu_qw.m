function [x, P] = tu_qw(x, P, omega, T, Rw)
%   time update function
%   Inputs:
%     x     - current state [q; b_omega; b_acc] (10×1)
%     P     - current covarience matrix (10×10)
%     omega - measured angular speed from gyro (3×1)
%     T     - sampling time
%     Rw    - process noise covariance matrix (9×9)
%
%   Outputs:
%     x     - predicted states
%     P     - predicted covarience matrix

    % state_vector
    q = x(1:4);         % quaternion
    b_omega = x(5:7);   % gyro bias
    b_acc = x(8:10);    % acc bias
    
    % true gyro angular speed
    omega_true = omega - b_omega;
    
    % 1. state prediction --------------------------------------------------------------
    % quaternion prediction
    F_omega = eye(4) + 0.5*T*Somega(omega_true);
    q_pred = F_omega * q;
    q_pred = mu_normalizeQ(q_pred);
    
    % bias prediction 
    b_omega_pred = b_omega;  % b_k^ω = b_{k-1}^ω
    b_acc_pred = b_acc;      % b_k^a = b_{k-1}^a
    
    % predicted states
    x_pred = [q_pred; b_omega_pred; b_acc_pred];
    
    % 2. covariance prediction ------------------------------------------------------------
    Gq = Gqfunc(q, T);
  
    % matrix A (10×10)
    A11 = F_omega;      % ∂q/∂q (4×4)
    A12 = -Gq;          % ∂q/∂b_omega (4×3)
    A13 = zeros(4,3);   % ∂q/∂b_acc (4×3)
    
    A21 = zeros(3,4);   % ∂b_omega/∂q (3×4)
    A22 = eye(3);       % ∂b_omega/∂b_omega (3×3)
    A23 = zeros(3,3);   % ∂b_omega/∂b_acc (3×3)
    
    A31 = zeros(3,4);   % ∂b_acc/∂q (3×4)
    A32 = zeros(3,3);   % ∂b_acc/∂b_omega (3×3)
    A33 = eye(3);       % ∂b_acc/∂b_acc (3×3)
    
    A = [A11, A12, A13;
         A21, A22, A23;
         A31, A32, A33];
    
    % matrix B (10×9)
    B11 = Gq;           % ∂q/∂v_omega (4×3)
    B12 = zeros(4,3);   % ∂q/∂w_omega (4×3)
    B13 = zeros(4,3);   % ∂q/∂w_acc (4×3)
    
    B21 = zeros(3,3);   % ∂b_omega/∂v_omega (3×3)
    B22 = eye(3);       % ∂b_omega/∂w_omega (3×3)
    B23 = zeros(3,3);   % ∂b_omega/∂w_acc (3×3)
    
    B31 = zeros(3,3);   % ∂b_acc/∂v_omega (3×3)
    B32 = zeros(3,3);   % ∂b_acc/∂w_omega (3×3)
    B33 = eye(3);       % ∂b_acc/∂w_acc (3×3)
    
    B = [B11, B12, B13;
         B21, B22, B23;
         B31, B32, B33];
    
    % covariance matrix: P_k = A*P_{k-1}*A' + B*Rw*B'
    P_pred = A * P * A' + B * Rw * B';
    
    % final results
    x = x_pred;
    P = P_pred;

end