function G_arm = arm_dynamics_tf(K_m, J, B)
%% ARM_DYNAMICS_TF 机械臂动力学传递函数建模
%   建立单关节机械臂的简化动力学传递函数模型
%
%   模型假设:
%     - 机械臂为理想刚体
%     - 关节处存在粘性阻尼
%     - 电机模型简化为比例环节
%
%   G(s) = K_m / (J * s^2 + B * s)
%
%   输入参数:
%     K_m - 电机转矩系数 (N·m/V)
%     J   - 转动惯量 (kg·m^2)
%     B   - 粘性阻尼系数 (N·m·s/rad)
%
%   返回值:
%     G_arm - 机械臂传递函数 (tf 对象)
%
%   示例:
%     J = 0.5 * 0.3^2;   % m=0.5kg, L=0.3m
%     G1 = arm_dynamics_tf(0.8, J, 0.05);

    num = [K_m];
    den = [J, B, 0];
    G_arm = tf(num, den);
end
