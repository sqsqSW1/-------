function snr_db = calc_snr(signal, noise)
%% CALC_SNR 信噪比计算
%   计算信号的信噪比 (Signal-to-Noise Ratio)
%
%   SNR = 20 * log10( RMS(signal) / RMS(noise) )
%
%   输入参数:
%     signal - 原始信号向量
%     noise  - 噪声信号向量
%
%   返回值:
%     snr_db - 信噪比 (dB)
%
%   示例:
%     snr = calc_snr(y_clean, y_clean - y_noisy);

    snr_db = 20 * log10(rms(signal) / max(rms(noise), eps));
end
