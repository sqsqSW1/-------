function [q_signal, q_error, q_step] = simulate_adc(signal, v_ref, bits)
%% SIMULATE_ADC ADC 量化仿真
%   模拟模数转换器的量化过程
%
%   输入参数:
%     signal - 原始模拟信号
%     v_ref  - ADC 参考电压 (V)
%     bits   - ADC 位数
%
%   返回值:
%     q_signal - 量化后的信号
%     q_error  - 量化误差
%     q_step   - 量化步长 / 电压分辨率 (V)
%
%   示例:
%     [y_q, err, step] = simulate_adc(y_sensor, 5.0, 12);

    q_step = v_ref / (2^bits - 1);
    q_signal = round(signal / q_step) * q_step;
    q_error = signal - q_signal;
end
