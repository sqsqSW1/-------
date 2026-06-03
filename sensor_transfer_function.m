function G_sensor = sensor_transfer_function(sensitivity, time_constant)
%% SENSOR_TRANSFER_FUNCTION 传感器传递函数建模
%   建立角度传感器的一阶惯性环节传递函数模型
%
%   G(s) = K / (tau * s + 1)
%
%   输入参数:
%     sensitivity   - 传感器灵敏度 (V/rad)
%     time_constant - 传感器时间常数 (s)
%
%   返回值:
%     G_sensor - 传感器传递函数 (tf 对象)
%
%   示例:
%     G2 = sensor_transfer_function(1.0, 0.01);

    num = [sensitivity];
    den = [time_constant, 1];
    G_sensor = tf(num, den);
end
