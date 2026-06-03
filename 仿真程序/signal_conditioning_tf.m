function G_cond = signal_conditioning_tf(gain, fc, damping)
%% SIGNAL_CONDITIONING_TF 信号调理电路传递函数建模
%   建立放大 + 二阶低通滤波器的传递函数模型
%
%   G(s) = K * omega_n^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2)
%
%   输入参数:
%     gain    - 放大器增益
%     fc      - 低通滤波器截止频率 (Hz)
%     damping - 阻尼比（可选，默认 0.707 = Butterworth 特性）
%
%   返回值:
%     G_cond - 信号调理传递函数 (tf 对象)
%
%   示例:
%     G3 = signal_conditioning_tf(10, 100, 0.707);

    if nargin < 3
        damping = 0.707;  % 默认 Butterworth 滤波器
    end
    omega_n = 2 * pi * fc;
    num = [gain * omega_n^2];
    den = [1, 2*damping*omega_n, omega_n^2];
    G_cond = tf(num, den);
end
