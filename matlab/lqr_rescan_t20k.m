function lqr_rescan_t20k()
% T20K 辨识植物全增益重推（2026-11-02，C 路线核心步）。
% 背景：C1 释放辨识 J_rod=1.95e-3（几何先验×9，磁码盘支架），现役 K1/K2
% （17751/10000）在该真实植物上线性不稳（+6.6 正实极点）——现役组一直在靠
% 死区/摩擦/非线性"非线性稳定"。本扫描在辨识植物上重推完整 LQR，
% 输出 [K1 K2 K3 K4] Q3.13 + 量化闭环裕度，板级只换运行时参数即可 A/B。
[A, B, P] = j280_linear_model(j280_params_t20k_identified());
fid = fopen(fullfile(fileparts(mfilename('fullpath')), '..', 'logs', '_lqr_rescan_t20k.txt'), 'w');
assert(fid >= 0); cleanup = onCleanup(@()fclose(fid));
fprintf(fid, '%6s %6s %8s %8s %8s %8s %9s %7s %s\n', ...
    'q_th', 'q_td', 'K1', 'K2', 'K3', 'K4', 'rho', 's16', '备注');
fprintf(fid, '%s\n', repmat('-', 1, 88));
% 扫描轴：θ臂位置权重（POS 需要大）、α 位置权重钉 8（摆杆直立整形）、
% 速度权重扫（阻尼来源）。R=1。
for q_th = [0.03 0.3 1.0 3.0 10.0]
    for q_td = [0.01 0.05 0.2 1.0]
        Q = diag([q_th, 8, 0.05, q_td]); R = 1;
        K = lqr(A, B, Q, R);
        k = -K([2 4 1 3]) * (P.duty_full / P.Vbus) .* pi ./ [18000 1800 18000 1800];
        kq = round(k * 2^13); kq = kq(:)';
        % 量化闭环校核（同 lqr_to_rtl 的逆变换）
        K_A = -kq([3 1 4 2]) / 2^13 / (P.duty_full / P.Vbus) ...
              .* [18000 18000 1800 1800] / pi;
        eg = eig(A - B * K_A);
        rho = max(abs(eg)); ok = all(real(eg) < 0);
        s16 = 'OK'; if any(abs(kq) >= 32768), s16 = '溢出'; end
        v = 'OK';
        if strcmp(s16, '溢出'), v = '溢出';
        elseif ~ok, v = '量化不稳';
        elseif rho > 0.998, v = '裕度不足';
        end
        fprintf(fid, '%6.2f %6.2f %8d %8d %8d %8d %9.6f %7s %s\n', ...
            q_th, q_td, kq(1), kq(2), kq(3), kq(4), rho, s16, v);
    end
end
% 对照：现役组
K_A = -[1200 17751 12000 10000] / 2^13 / (P.duty_full / P.Vbus) ...
      .* [18000 18000 1800 1800] / pi;
eg = eig(A - B * K_A);
fprintf(fid, '现役(17751,10000,1200,12000): rho=%.4f ok=%d\n', ...
    max(abs(eg)), double(all(real(eg) < 0)));
fprintf('%s\n', fileread(fullfile(fileparts(mfilename('fullpath')), '..', 'logs', '_lqr_rescan_t20k.txt')));
end
