function eulerAngles = quat2euler(q)
% QUAT2EULER 将四元数批量转换为欧拉角（Z-Y-X顺序，即yaw-pitch-roll）
%   输入：q - 4×N的四元数矩阵，每列代表一个四元数[w;x;y;z]
%   输出：eulerAngles - 3×N的欧拉角矩阵，单位为弧度，顺序为[roll; pitch; yaw]

% 检查输入维度
if size(q,1) ~= 4
    error('输入必须是4×N的四元数矩阵');
end

N = size(q,2);  % 获取四元数数量
eulerAngles = zeros(3, N);  % 预分配输出矩阵

for i = 1:N
    % 提取当前四元数分量
    w = q(1,i);
    x = q(2,i);
    y = q(3,i);
    z = q(4,i);
    
    % 计算欧拉角（Z-Y-X顺序）
    % Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x^2 + y^2);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    % Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x);
    if abs(sinp) >= 1
        pitch = sign(sinp) * pi/2; % 使用90度，如果超出范围
    else
        pitch = asin(sinp);
    end
    
    % Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y^2 + z^2);
    yaw = atan2(siny_cosp, cosy_cosp);
    
    % 存储结果
    eulerAngles(:,i) = [roll; pitch; yaw];
end
end