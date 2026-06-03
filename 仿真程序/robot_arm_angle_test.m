%% ========================================================================
%  机械臂摆角测试系统 —— MATLAB 仿真程序
%  Robotic Arm Swing Angle Testing System — MATLAB Simulation
%
%  课程：测试技术 课程项目
%  系统描述：单关节机械臂摆角测试系统，包含传感器建模、信号调理、
%           传递函数分析、时域/频域响应及噪声影响分析。
%  适用对象：本科大三学生
%  运行方式：在 MATLAB 中直接运行本脚本，或分段执行各节代码。
% ========================================================================

clear; clc; close all;

%% ========================================================================
%  第一部分：系统参数定义与初始化
% ========================================================================
fprintf('=============================================================\n');
fprintf('  机械臂摆角测试系统 —— 仿真开始\n');
fprintf('=============================================================\n\n');

% ---------- 1.1 机械臂物理参数 ----------
L     = 0.30;      % 臂长 (m)
m     = 0.50;      % 末端等效质量 (kg)
g     = 9.81;      % 重力加速度 (m/s^2)
J     = m * L^2;   % 转动惯量 (kg·m^2)
B     = 0.05;      % 粘性阻尼系数 (N·m·s/rad)
K_m   = 0.8;       % 电机转矩系数 (N·m/V)

% ---------- 1.2 角度传感器参数（旋转电位器） ----------
%  选用旋转电位器作为角度传感器，可建模为一阶惯性环节
tau_s = 0.01;      % 传感器时间常数 (s)
K_s   = 1.0;       % 传感器灵敏度 (V/rad)

% ---------- 1.3 信号调理电路参数 ----------
%  包括放大器和二阶低通滤波器
K_a   = 10;        % 放大器增益
f_c   = 100;       % 低通滤波器截止频率 (Hz)
omega_n = 2 * pi * f_c;  % 固有频率 (rad/s)
zeta  = 0.707;     % 阻尼比（Butterworth 特性）

% ---------- 1.4 ADC 参数 ----------
ADC_bits = 12;             % ADC 位数
V_ref    = 5.0;            % 参考电压 (V)
ADC_res  = V_ref / (2^ADC_bits - 1);  % 分辨率 (V)
ADC_q    = ADC_res / (K_s * K_a);      % 角度量化误差 (rad)

% ---------- 1.5 仿真参数 ----------
fs    = 1000;      % 采样频率 (Hz)
Ts    = 1/fs;      % 采样周期 (s)
T_end = 5;         % 仿真总时长 (s)
t     = 0:Ts:T_end; % 时间向量
N     = length(t);  % 采样点数

fprintf('系统参数初始化完成。\n');
fprintf('  臂长 L = %.2f m, 质量 m = %.2f kg\n', L, m);
fprintf('  转动惯量 J = %.4f kg·m^2\n', J);
fprintf('  传感器时间常数 tau_s = %.3f s\n', tau_s);
fprintf('  滤波器截止频率 f_c = %.1f Hz\n', f_c);
fprintf('  ADC 分辨率 = %.2f mV\n\n', ADC_res * 1000);

%% ========================================================================
%  第二部分：被测信号建模（时域与频域分析）
% ========================================================================
fprintf('第二部分：被测信号建模...\n');

% ---------- 2.1 生成测试信号 ----------
% 输入信号1：阶跃信号（模拟机械臂从0°转到45°）
theta_step_amp = pi/4;  % 45° = π/4 rad
u_step = theta_step_amp * ones(size(t));
u_step(t < 0.5) = 0;   % 在 t=0.5s 时刻施加阶跃

% 输入信号2：正弦信号（模拟机械臂周期摆动）
f_in = 0.5;            % 输入频率 (Hz)
A_in = pi/6;           % 振幅 30° = π/6 rad
u_sine = A_in * sin(2 * pi * f_in * t);

% 输入信号3：扫频信号（Chirp，用于频率特性分析）
f0 = 0.1;  f1 = 10;    % 扫频范围 0.1~10 Hz
u_chirp = (pi/6) * chirp(t, f0, T_end, f1);

fprintf('  生成 3 种测试信号：阶跃、正弦、扫频 (Chirp)\n');

% ---------- 2.2 输入信号频谱分析 ----------
freq = (0:N/2-1) * fs / N;  % 频率轴

U_step_fft = abs(fft(u_step)) / N;
U_sine_fft = abs(fft(u_sine)) / N;
U_chirp_fft = abs(fft(u_chirp)) / N;

fprintf('  输入信号 FFT 计算完成。\n\n');

%% ========================================================================
%  第三部分：测试系统各环节传递函数建模
% ========================================================================
fprintf('第三部分：传递函数建模...\n');

% ---------- 3.1 机械臂动力学传递函数 G1(s) ----------
%  G1(s) = K_m / (J*s^2 + B*s)
%  输入：电机电压，输出：关节角位移
num_G1 = [K_m];
den_G1 = [J, B, 0];
G1 = tf(num_G1, den_G1);
fprintf('  G1(s) — 机械臂动力学：G1(s) = %.2f / (%.4f s^2 + %.2f s)\n', K_m, J, B);

% ---------- 3.2 角度传感器传递函数 G2(s) ----------
%  G2(s) = K_s / (tau_s * s + 1)    （一阶惯性环节）
%  输入：实际角位移，输出：电压信号
num_G2 = [K_s];
den_G2 = [tau_s, 1];
G2 = tf(num_G2, den_G2);
fprintf('  G2(s) — 电位器传感器：G2(s) = %.1f / (%.2f s + 1)\n', K_s, tau_s);

% ---------- 3.3 信号调理电路传递函数 G3(s) ----------
%  G3(s) = K_a * omega_n^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2)
%  二阶低通滤波器 + 放大器
num_G3 = [K_a * omega_n^2];
den_G3 = [1, 2*zeta*omega_n, omega_n^2];
G3 = tf(num_G3, den_G3);
fprintf('  G3(s) — 信号调理（放大+低通滤波）\n');

% ---------- 3.4 ADC 环节 ----------
%  ADC 建模为零阶保持器 + 量化
%  采样保持传递函数：H_zoh(s) = (1 - exp(-Ts*s)) / s

% ---------- 3.5 系统总传递函数 ----------
%  G_total(s) = G1(s) * G2(s) * G3(s)
G_total = G1 * G2 * G3;
fprintf('  G_total(s) — 系统总传递函数（不含 ADC）已建立。\n\n');

% ---------- 3.6 显示传递函数 ----------
fprintf('---------- 系统各环节传递函数 ----------\n');
fprintf('机械臂 G1(s):\n');  G1
fprintf('传感器 G2(s):\n');  G2
fprintf('信号调理 G3(s):\n'); G3
fprintf('系统总传递函数 G_total(s):\n'); G_total
fprintf('----------------------------------------\n\n');

%% ========================================================================
%  第四部分：时域响应分析
% ========================================================================
fprintf('第四部分：时域响应分析...\n');

% ---------- 4.1 阶跃响应 ----------
figure('Name', '时域响应分析', 'Position', [100, 100, 1000, 700]);

% 机械臂环节阶跃响应
subplot(2,3,1);
step(G1, t);
title('(a) 机械臂 G1(s) 阶跃响应');
xlabel('时间 (s)'); ylabel('角位移 (rad)');
grid on;

% 传感器环节阶跃响应
subplot(2,3,2);
step(G2, 0:0.001:0.1);
title('(b) 传感器 G2(s) 阶跃响应');
xlabel('时间 (s)'); ylabel('电压 (V)');
grid on;

% 信号调理环节阶跃响应
subplot(2,3,3);
step(G3, 0:0.001:0.05);
title('(c) 信号调理 G3(s) 阶跃响应');
xlabel('时间 (s)'); ylabel('电压 (V)');
grid on;

% 系统总阶跃响应
subplot(2,3,4:6);
step(G_total, t);
title('(d) 系统总传递函数 G_{total}(s) 阶跃响应');
xlabel('时间 (s)'); ylabel('输出电压 (V)');
grid on;

sgtitle('图1：测试系统各环节阶跃响应');

% ---------- 4.2 提取时域性能指标 ----------
info_total = stepinfo(G_total);
fprintf('  系统阶跃响应性能指标:\n');
fprintf('    上升时间 (Rise Time):  %.4f s\n', info_total.RiseTime);
fprintf('    调节时间 (Settling Time): %.4f s\n', info_total.SettlingTime);
fprintf('    超调量 (Overshoot):   %.2f %%\n', info_total.Overshoot);
fprintf('    峰值时间 (Peak Time):  %.4f s\n\n', info_total.PeakTime);

%% ========================================================================
%  第五部分：频域响应分析
% ========================================================================
fprintf('第五部分：频域响应分析...\n');

figure('Name', '频域响应分析', 'Position', [150, 150, 1000, 800]);

% ---------- 5.1 Bode 图 ----------
subplot(2,2,1);
bode(G_total);
title('(a) 系统总传递函数 Bode 图');
grid on;

% ---------- 5.2 各环节频率响应对比 ----------
subplot(2,2,2);
w = logspace(-1, 4, 1000);  % 频率范围 0.1 ~ 10000 rad/s
[mag1, ~] = bode(G1, w); mag1 = squeeze(mag1);
[mag2, ~] = bode(G2, w); mag2 = squeeze(mag2);
[mag3, ~] = bode(G3, w); mag3 = squeeze(mag3);
[mag_total, ~] = bode(G_total, w); mag_total = squeeze(mag_total);

semilogx(w, 20*log10(mag1), 'b-', 'LineWidth', 1.2); hold on;
semilogx(w, 20*log10(mag2), 'r--', 'LineWidth', 1.2);
semilogx(w, 20*log10(mag3), 'g-.', 'LineWidth', 1.2);
semilogx(w, 20*log10(mag_total), 'k-', 'LineWidth', 2);
xlabel('频率 (rad/s)'); ylabel('幅值 (dB)');
title('(b) 各环节幅频特性对比');
legend('G1 机械臂', 'G2 传感器', 'G3 信号调理', 'G_{total} 总系统', ...
       'Location', 'southwest');
grid on;

% ---------- 5.3 Nyquist 图 ----------
subplot(2,2,3);
nyquist(G_total);
title('(c) Nyquist 图');
grid on; axis equal;

% ---------- 5.4 系统带宽分析 ----------
subplot(2,2,4);
[mag_w, ~, w_out] = bode(G_total, w);
mag_w_dB = 20 * log10(squeeze(mag_w));
semilogx(w_out, mag_w_dB, 'b-', 'LineWidth', 1.5); hold on;
% 标注 -3dB 带宽
bw = NaN;  % 初始化为 NaN，防止后续引用未定义变量
idx_3dB = find(mag_w_dB <= max(mag_w_dB) - 3, 1);
if ~isempty(idx_3dB)
    bw = w_out(idx_3dB);
    plot([w_out(1), bw], [max(mag_w_dB)-3, max(mag_w_dB)-3], 'r--', 'LineWidth', 1);
    plot([bw, bw], [min(mag_w_dB)-10, max(mag_w_dB)-3], 'r--', 'LineWidth', 1);
    text(bw*1.5, max(mag_w_dB)-2, sprintf('-3dB @ %.2f rad/s (%.2f Hz)', bw, bw/(2*pi)), ...
         'Color', 'r', 'FontSize', 9);
end
xlabel('频率 (rad/s)'); ylabel('幅值 (dB)');
title('(d) 系统带宽分析');
grid on;

sgtitle('图2：测试系统频域分析');

fprintf('  频域分析完成。\n');
if ~isempty(idx_3dB)
    fprintf('  系统 -3dB 带宽: %.2f rad/s (%.2f Hz)\n\n', bw, bw/(2*pi));
end

%% ========================================================================
%  第六部分：系统仿真 —— 时域信号响应
% ========================================================================
fprintf('第六部分：系统时域仿真...\n');

% ---------- 6.1 阶跃输入响应 ----------
y_step = lsim(G_total, u_step, t);

% ---------- 6.2 正弦输入响应 ----------
y_sine = lsim(G_total, u_sine, t);

% ---------- 6.3 扫频输入响应 ----------
y_chirp = lsim(G_total, u_chirp, t);

figure('Name', '系统时域仿真', 'Position', [200, 200, 1000, 700]);

% 阶跃响应
subplot(3,2,1);
plot(t, u_step * 180/pi, 'b--', 'LineWidth', 1); hold on;
plot(t, y_step / (K_s*K_a) * 180/pi, 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('角度 (deg)');
title('(a) 阶跃输入响应');
legend('输入角度', '测量输出', 'Location', 'best');
grid on;

% 正弦响应
subplot(3,2,2);
plot(t, u_sine * 180/pi, 'b--', 'LineWidth', 1); hold on;
plot(t, y_sine / (K_s*K_a) * 180/pi, 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('角度 (deg)');
title('(b) 正弦输入响应');
legend('输入角度', '测量输出', 'Location', 'best');
grid on;

% 阶跃响应误差
subplot(3,2,3);
error_step = (u_step - y_step / (K_s*K_a)) * 180/pi;
plot(t, error_step, 'm-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('角度误差 (deg)');
title('(c) 阶跃响应跟踪误差');
grid on;

% 正弦响应误差
subplot(3,2,4);
error_sine = (u_sine - y_sine / (K_s*K_a)) * 180/pi;
plot(t, error_sine, 'm-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('角度误差 (deg)');
title('(d) 正弦响应跟踪误差');
grid on;

% 扫频响应
subplot(3,2,5:6);
plot(t, u_chirp * 180/pi, 'b--', 'LineWidth', 1); hold on;
plot(t, y_chirp / (K_s*K_a) * 180/pi, 'r-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('角度 (deg)');
title('(e) 扫频信号响应');
legend('输入角度', '测量输出', 'Location', 'best');
grid on;

sgtitle('图3：系统对不同输入信号的时域响应');

fprintf('  时域仿真完成。\n\n');

%% ========================================================================
%  第七部分：噪声影响分析
% ========================================================================
fprintf('第七部分：噪声影响分析...\n');

% ---------- 7.1 添加测量噪声 ----------
%  模拟实际测量环境中的噪声
noise_std = 0.02;  % 噪声标准差 (V) — 对应约 0.1° 的角度噪声
noise_meas = noise_std * randn(size(t));  % 高斯白噪声

%  量化噪声（ADC 量化误差，近似为均匀分布白噪声）
q_noise_std = ADC_res / sqrt(12);  % 量化噪声标准差
q_noise = q_noise_std * randn(size(t));

% 总噪声
total_noise = noise_meas + q_noise;

% ---------- 7.2 带噪声的系统响应 ----------
y_step_noisy = y_step + total_noise;
y_sine_noisy = y_sine + total_noise;

figure('Name', '噪声影响分析', 'Position', [250, 250, 1000, 700]);

% 阶跃响应（含噪声）
subplot(2,2,1);
plot(t, y_step / (K_s*K_a) * 180/pi, 'b-', 'LineWidth', 1); hold on;
plot(t, y_step_noisy / (K_s*K_a) * 180/pi, 'r.', 'MarkerSize', 3);
xlabel('时间 (s)'); ylabel('角度 (deg)');
title('(a) 阶跃响应（含测量噪声）');
legend('无噪声', '含噪声', 'Location', 'best');
grid on;

% 正弦响应（含噪声）
subplot(2,2,2);
plot(t, y_sine / (K_s*K_a) * 180/pi, 'b-', 'LineWidth', 1); hold on;
plot(t, y_sine_noisy / (K_s*K_a) * 180/pi, 'r.', 'MarkerSize', 3);
xlabel('时间 (s)'); ylabel('角度 (deg)');
title('(b) 正弦响应（含测量噪声）');
legend('无噪声', '含噪声', 'Location', 'best');
grid on;

% 噪声统计直方图
subplot(2,2,3);
histogram(total_noise * 1000, 40, 'FaceColor', [0.6 0.2 0.2], 'EdgeColor', 'w');
xlabel('噪声幅值 (mV)'); ylabel('频次');
title('(c) 测量噪声分布直方图');
grid on;

% 信噪比分析
subplot(2,2,4);
SNR_step = 20 * log10(rms(y_step) / rms(total_noise));
SNR_sine = 20 * log10(rms(y_sine) / rms(total_noise));
bar([SNR_step, SNR_sine], 'FaceColor', [0.2 0.4 0.7]);
set(gca, 'XTickLabel', {'阶跃输入', '正弦输入'});
ylabel('SNR (dB)');
title('(d) 系统输出信噪比');
text(1:2, [SNR_step, SNR_sine], ...
     {sprintf('%.1f dB', SNR_step), sprintf('%.1f dB', SNR_sine)}, ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
grid on;

sgtitle('图4：测量噪声对系统的影响分析');

fprintf('  噪声分析完成。\n');
fprintf('    测量噪声标准差: %.1f mV\n', noise_std * 1000);
fprintf('    量化噪声标准差: %.2f mV\n', q_noise_std * 1000);
fprintf('    阶跃输入 SNR: %.1f dB\n', SNR_step);
fprintf('    正弦输入 SNR: %.1f dB\n\n', SNR_sine);

%% ========================================================================
%  第八部分：系统稳定性分析
% ========================================================================
fprintf('第八部分：系统稳定性分析...\n');

% ---------- 8.1 极点-零点图 ----------
figure('Name', '系统稳定性分析', 'Position', [300, 300, 800, 600]);

subplot(2,2,1);
pzmap(G_total);
title('(a) 系统极点-零点分布图');
grid on;

% ---------- 8.2 极点数值 ----------
poles = pole(G_total);
subplot(2,2,2);
plot(real(poles), imag(poles), 'x', 'MarkerSize', 12, 'LineWidth', 2, ...
     'Color', 'r');
xlabel('实部'); ylabel('虚部');
title('(b) 系统极点（标注）'); grid on; hold on;
xline(0, 'k--'); yline(0, 'k--');
axis equal;
x_lim = max(max(abs(real(poles)))*1.5, 0.5);
y_lim = max(max(abs(imag(poles)))*1.5, 0.5);
xlim([-x_lim, x_lim]);
ylim([-y_lim, y_lim]);

% ---------- 8.3 稳定性判断 ----------
subplot(2,2,3:4);
axis off;
text(0.1, 0.8, '系统稳定性分析结果', 'FontSize', 14, 'FontWeight', 'bold');
text(0.1, 0.6, sprintf('系统极点个数: %d', length(poles)), 'FontSize', 11);

stable_flag = true;
for i = 1:length(poles)
    text(0.1, 0.55 - i*0.08, ...
         sprintf('  极点 %d: s = %.4f %+.4fj', i, real(poles(i)), imag(poles(i))), ...
         'FontSize', 10);
    if real(poles(i)) >= 0
        stable_flag = false;
    end
end

if stable_flag
    text(0.1, 0.55 - length(poles)*0.08 - 0.1, ...
         '✓ 结论：系统稳定。所有极点均位于 s 平面左半平面。', ...
         'FontSize', 12, 'Color', [0 0.5 0], 'FontWeight', 'bold');
else
    text(0.1, 0.55 - length(poles)*0.08 - 0.1, ...
         '✗ 结论：系统不稳定！存在右半平面极点。', ...
         'FontSize', 12, 'Color', 'r', 'FontWeight', 'bold');
end

sgtitle('图5：系统稳定性分析');

if stable_flag
    fprintf('  系统是稳定的（所有极点位于左半平面）。\n\n');
else
    fprintf('  系统是不稳定的！\n\n');
end

%% ========================================================================
%  第九部分：ADC 量化效应分析
% ========================================================================
fprintf('第九部分：ADC 量化效应分析...\n');

% 模拟 ADC 量化过程
y_sine_quantized = round(y_sine / ADC_res) * ADC_res;
quantization_error = y_sine - y_sine_quantized;

figure('Name', 'ADC 量化效应', 'Position', [350, 350, 900, 500]);

subplot(2,2,1);
plot(t, y_sine, 'b-', 'LineWidth', 1); hold on;
stairs(t, y_sine_quantized, 'r-', 'LineWidth', 0.8);
xlim([1, 1.05]);  % 局部放大
xlabel('时间 (s)'); ylabel('电压 (V)');
title('(a) ADC 量化效果（局部放大）');
legend('原始信号', '量化后信号', 'Location', 'best');
grid on;

subplot(2,2,2);
plot(t, quantization_error * 1000, 'm-', 'LineWidth', 0.8);
xlabel('时间 (s)'); ylabel('量化误差 (mV)');
title('(b) ADC 量化误差');
grid on;

subplot(2,2,3);
histogram(quantization_error * 1000, 30, 'FaceColor', [0.5 0.2 0.5], ...
          'EdgeColor', 'w');
xlabel('量化误差 (mV)'); ylabel('频次');
title('(c) 量化误差分布');
grid on;

subplot(2,2,4);
axis off;
text(0.1, 0.9, 'ADC 量化参数', 'FontSize', 13, 'FontWeight', 'bold');
text(0.1, 0.75, sprintf('ADC 位数: %d bits', ADC_bits), 'FontSize', 11);
text(0.1, 0.65, sprintf('参考电压: %.2f V', V_ref), 'FontSize', 11);
text(0.1, 0.55, sprintf('电压分辨率: %.3f mV', ADC_res * 1000), 'FontSize', 11);
text(0.1, 0.45, sprintf('角度分辨率: %.4f °', ADC_q * 180/pi), 'FontSize', 11);
text(0.1, 0.35, sprintf('量化噪声 RMS: %.3f mV', rms(quantization_error) * 1000), ...
     'FontSize', 11);

sgtitle('图6：ADC 量化效应分析');

fprintf('  ADC 分析完成。\n');
fprintf('    电压分辨率: %.3f mV\n', ADC_res * 1000);
fprintf('    角度分辨率: %.4f °\n', ADC_q * 180/pi);
fprintf('    量化噪声 RMS: %.3f mV\n\n', rms(quantization_error) * 1000);

%% ========================================================================
%  第十部分：综合结果汇总
% ========================================================================
fprintf('=============================================================\n');
fprintf('  仿真结果汇总\n');
fprintf('=============================================================\n');
fprintf('  1. 系统结构:\n');
fprintf('     机械臂动力学(G1) → 电位器传感器(G2) → 信号调理(G3) → ADC\n');
fprintf('  2. 阶跃响应性能:\n');
fprintf('     上升时间: %.4f s\n', info_total.RiseTime);
fprintf('     调节时间: %.4f s\n', info_total.SettlingTime);
fprintf('     超调量:   %.2f %%\n', info_total.Overshoot);
if ~isnan(bw)
    fprintf('  3. 系统带宽 (-3dB): %.2f rad/s (%.2f Hz)\n', bw, bw/(2*pi));
else
    fprintf('  3. 系统带宽 (-3dB): 超出分析范围\n');
end
fprintf('  4. 稳态误差 (阶跃): %.4f °\n', ...
        abs(mean(error_step(end-round(0.1*N):end))));
fprintf('  5. 信噪比:\n');
fprintf('     阶跃输入: %.1f dB\n', SNR_step);
fprintf('     正弦输入: %.1f dB\n', SNR_sine);
fprintf('  6. ADC 角度分辨率: %.4f °\n', ADC_q * 180/pi);
fprintf('  7. 系统稳定性: %s\n', repmat('稳定', stable_flag));
fprintf('=============================================================\n');

fprintf('\n仿真程序运行完毕。所有图形窗口已生成。\n');
fprintf('请查看各 Figure 窗口以获取详细分析图表。\n');
