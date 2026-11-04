function pos_pair_scan_t20k()
% POS 对重推（2026-11-02，修八架构 + T20K 辨识植物）。
% 与 pos_pair_scan.m 差别：①植物换 T20K 实机辨识 profile（C1+C2）；
% ②验收基准 K1/K2 = 现役板上值（Q3.13：17751/10000）；
% ③输出含与现役 POS 对 (1200,12000) 的裕度对照。
[A, B, P] = j280_linear_model(j280_params_t20k_identified());
R = 1;
fid = fopen(fullfile(fileparts(mfilename('fullpath')), '..', 'logs', '_pos_pair_scan_t20k.txt'), 'w');
assert(fid >= 0); cleanup = onCleanup(@()fclose(fid));
fprintf(fid, '%8s %6s %8s %8s %9s %7s %8s %s\n', 'q_theta', 'q_tdot', 'K3', 'K4', 'rho', 'eigmax', 's16', '判定');
fprintf(fid, '%s\n', repmat('-', 1, 80));
for qp = [0.1 0.3 1.0 3.0 10.0 30.0]
    for td = [0.01 0.02 0.05]
        Q = diag([qp, 2, 0.05, td]);
        K = lqr(A, B, Q, R);
        k = -K([2 4 1 3]) * (P.duty_full / P.Vbus) .* pi ./ [18000 1800 18000 1800];
        kq = round(k * 2^13); kq = kq(:)';
        K3 = kq(3); K4 = kq(4);
        kq(1) = 17751; kq(2) = 10000;   % 现役 K1/K2 不动
        [ok, rho, eigmax] = check_candidate_t20k(kq, A, B, P);
        s16 = 'OK'; if abs(K3) >= 32768 || abs(K4) >= 32768, s16 = '溢出'; end
        v = 'OK';
        if strcmp(s16, '溢出'), v = 's16溢出';
        elseif ~ok, v = '闭环不稳';
        elseif eigmax >= 0, v = '不稳';
        end
        fprintf(fid, '%8.1f %6.2f %8d %8d %9.6f %7.3f %8s %s\n', ...
            qp, td, K3, K4, eigmax, rho, s16, v);
    end
end
% 现役对对照
kq = [17751 10000 1200 12000];
[ok, rho, eigmax] = check_candidate_t20k(kq, A, B, P);
fprintf(fid, '现役(1200,12000): rho=%.6f eigmax=%.3f ok=%d\n', rho, eigmax, ok);
fprintf('%s\n', fileread(fullfile(fileparts(mfilename('fullpath')), '..', 'logs', '_pos_pair_scan_t20k.txt')));
end

function [ok, rho, eigmax] = check_candidate_t20k(kq, A, B, P)
% RTL 增益链：duty = -(Kq/8192)·[alpha, alpha_dot, theta_e, theta_dot]，
% RTL 状态序换算 SI 后须【重排回 A 的状态序 [θ臂, α摆, θ̇臂, α̇]】再算闭环。
S = diag([pi/18000, pi/1800, pi/18000, pi/1800]);   % 对应 RTL 序
Ksi_rtl = (kq / 2^13) * S;               % RTL 序 SI 增益 [k_a, k_ad, k_te, k_td]
% RTL→A 序映射：A1=θ(=RTL3), A2=α(=RTL1), A3=θ̇(=RTL4), A4=α̇(=RTL2)
Ksi = zeros(1, 4);
Ksi(1) = Ksi_rtl(3); Ksi(2) = Ksi_rtl(1);
Ksi(3) = Ksi_rtl(4); Ksi(4) = Ksi_rtl(2);
Acl = A - B * Ksi;
eg = eig(Acl);
rho = max(abs(eg)); eigmax = max(real(eg));
ok = all(real(eg) < 0);
end
