function pos_data_analysis()
% pos_data_analysis.m — 用 11-4 夜/凌晨的 POS 保持段数据做相对辨识（不依赖绝对模型尺度）
% 数据：matlab/data/pos_*.csv（500Hz，POS 保持段，已剔入场瞬态）
% 输出：matlab/data/_analysis_out.txt + 三张 PNG
% 纪律：11-3 教训——绝对标度模型不可信；本脚本只做【闭环数据相对量】：
%   A. 0.25Hz 慢模态频率/阻尼（Hilbert 包络）——各段对比
%   B. pos_trim(θ) 直流补偿图谱 = dz LUT 残差 + 摩擦位置相关项 → 前馈建议
%   C. θ̇ 高频噪声与 K4 权限账 → 滤波器重设计空间
%   D. 综合建议（按预期 te 改善排序）

D = dir(fullfile(fileparts(mfilename('fullpath')), 'data', 'pos_*.csv'));
fid = fopen(fullfile(fileparts(mfilename('fullpath')), 'data', '_analysis_out.txt'), 'w');
assert(fid >= 0); cleanup = onCleanup(@()fclose(fid));
P = @(varargin) fprintf(fid, varargin{:});   % 同时打屏
scr = onCleanup(@() fclose('all'));
out = @(varargin) {fprintf(1, varargin{:}); P(varargin{:})};

segNames = {D.name};
nSeg = numel(D);
out('=== POS 数据相对分析（%d 段）===\n', nSeg);

%% A. 慢模态频率/阻尼
out('\n--- A. te 主模态（0.05-0.6Hz 带内主峰 + 包络阻尼）---\n');
out('%-34s %5s %7s %7s %9s %9s %9s\n', 'seg', 'n/s', 'f0_Hz', 'zeta', 'te_pp_deg', 'te_abs_deg', 'al_max_deg');
A = zeros(nSeg, 3);
for i = 1:nSeg
    T = readtable(fullfile(D(i).folder, D(i).name));
    te = T.theta_e / 100;                    % deg
    te = te - mean(te);
    fs = 500;
    n = numel(te);
    % Welch 均匀分段周期图（手写，8 段 50% 重叠）
    L = 2 ^ nextpow2(min(20000, floor(n / 4)));
    f = (0:L/2) * fs / L;
    seg = floor(n / 4) * 2;                  % 段长
    nseg = max(1, floor((n - seg) / (seg / 2)) + 1);
    Pxx = zeros(L / 2 + 1, 1);
    w = hamming(seg);
    for k = 1:nseg
        s0 = (k - 1) * seg / 2 + 1;
        x = te(s0:s0 + seg - 1) .* w;
        X = abs(fft(x, L)).^2 / (sum(w.^2) * fs);
        Pxx = Pxx + X(1:L/2+1);
    end
    Pxx = Pxx / nseg;
    band = f >= 0.05 & f <= 0.6;
    [~, im] = max(Pxx .* band');
    f0 = f(im);
    % 包络阻尼：FFT 希尔伯特（基 MATLAB 实现）
    if n > 4 / f0 * fs
        x = te(1:min(n, floor(30 * fs)));
        m = numel(x);
        X = fft(x);
        h = zeros(m, 1); h(1) = 1; h(2:floor(m/2)) = 2;
        if mod(m, 2) == 0, h(m/2+1) = 1; end
        env = abs(ifft(X .* h));
        % log 包络线性拟合（取上包络的 90 分位滑窗避免噪声拖偏）
        wl = round(fs * 0.4);
        nenv = floor(m / wl);
        tt = zeros(nenv, 1); ee = zeros(nenv, 1);
        for k = 1:nenv
            v = env((k-1)*wl+1 : k*wl);
            tt(k) = ((k-1)*wl + wl/2) / fs;
            vs = sort(v);                      % 手写 0.9 分位（免 Statistics 工具箱）
            ee(k) = vs(min(end, max(1, round(0.9 * numel(vs)))));
        end
        ok = ee > max(ee) * 0.05 & ee > 1e-6;
        if nnz(ok) >= 5
            p = polyfit(tt(ok), log(ee(ok)), 1);
            sigma = -p(1);                     % 1/s（衰减率）
            zeta = sigma / (2 * pi * f0);
        else
            zeta = NaN;
        end
    else
        zeta = NaN;
    end
    A(i, :) = [f0, zeta, n / fs];
    out('%-34s %5.0f %7.3f %7.3f %9.1f %9.1f %9.1f\n', strrep(D(i).name, 'pos_', ''), ...
        n / fs, f0, zeta, (max(T.theta_e) - min(T.theta_e)) / 100, ...
        mean(abs(T.theta_e)) / 100, max(abs(T.alpha_c01)) / 100);
end

%% B. pos_trim(θ) 直流补偿图谱（dz LUT 残差 + 摩擦位置项）
out('\n--- B. pos_trim(θ) 图谱（积分器被迫出的补偿=dz LUT 残差；IQ 限幅 ±600）---\n');
thAll = []; ptAll = []; teAll = [];
for i = 1:nSeg
    T = readtable(fullfile(D(i).folder, D(i).name));
    % 只取 |te|<30° 的样本（积分器门内，pos_trim 才代表位置相关直流需求）
    m = abs(T.theta_e) < 3000 & abs(T.alpha_c01) < 500;
    thAll = [thAll; mod(T.theta_c01(m) / 100, 360)]; %#ok<AGROW>
    ptAll = [ptAll; T.pos_trim(m)];              %#ok<AGROW>
    teAll = [teAll; T.theta_e(m) / 100];         %#ok<AGROW>
end
edges = 0:30:360;
[nc, ~] = histcounts(thAll, edges);
mu_pt = accumarray(min(floor(thAll / 30) + 1, 12), ptAll, [12 1], @mean, NaN);
sd_pt = accumarray(min(floor(thAll / 30) + 1, 12), ptAll, [12 1], @std, NaN);
mu_te = accumarray(min(floor(thAll / 30) + 1, 12), teAll, [12 1], @mean, NaN);
out('%6s %7s %9s %9s %9s\n', 'θ_bin', 'n', 'pos_trim', 'sd', 'te_mean_deg');
for k = 1:12
    out('%4d-%3d %7d %9.0f %9.0f %+9.2f\n', edges(k), edges(k+1), nc(k), ...
        mu_pt(k), sd_pt(k), mu_te(k));
end
out('pos_trim 饱和率: |pt|>=590 占 %.1f%%\n', 100 * mean(abs(ptAll) >= 590));
fig = figure('Visible', 'off', 'Position', [100 100 900 700]);
subplot(3, 1, 1);
plot(thAll, ptAll, '.', 'MarkerSize', 2); grid on;
ylabel('pos\_trim'); title('pos\_trim(θ) — dz LUT 残差图谱'); ylim([-650 650]);
subplot(3, 1, 2);
plot(thAll, teAll, '.', 'MarkerSize', 2); grid on;
ylabel('te (deg)'); title('te(θ)');
subplot(3, 1, 3);
bar(edges(1:end-1) + 15, mu_pt, 0.8); grid on;
xlabel('θ (deg)'); ylabel('mean pos\_trim');
saveas(fig, fullfile(fileparts(mfilename('fullpath')), 'data', '_pos_trim_map.png'));

%% C. θ̇ 高频噪声账 + 滤波器重设计空间
out('\n--- C. θ̇ 噪声账（K4 通道高频注入 vs duty 高频实测）---\n');
out('%-34s %9s %9s %9s %9s\n', 'seg', 'thdot_std', 'K4*thdot', 'duty_std', 'duty_hf_std');
for i = 1:nSeg
    T = readtable(fullfile(D(i).folder, D(i).name));
    td = T.theta_dot - movmean(T.theta_dot, 50);       % >1Hz 高频
    du = T.duty - movmean(T.duty, 50);
    k4 = median(T.k4_eff(T.k4_eff > 0));
    out('%-34s %9.1f %9.0f %9.0f %9.0f\n', strrep(D(i).name, 'pos_', ''), ...
        std(T.theta_dot) / 10, k4 * std(td) / 8192, std(T.duty), std(du));
end
out('注：K4·θ̇_hf/8192 为速度反馈对 duty 的高频注入估计；duty_hf_std 为实测高频。\n');

% 候选滤波器：二阶巴特沃斯 lp 6Hz（双线性手工系数，fs=500Hz）
% [b,a] = butter(2, 6/(fs/2)) → 手算系数（MATLAB 若有 Signal Toolbox 用 butter，否则用下面常数）
fs = 500; fc = 6;
wn = 2 * pi * fc / fs;
% 双线性二阶巴特沃斯：Ω = tan(wn/2)
W = tan(wn / 2);
K = W ^ 2 / (1 + sqrt(2) * W + W ^ 2);
b1f = [K, 2 * K, K]; a1f = [1, (2 * (W ^ 2 - 1)) / (1 + sqrt(2) * W + W ^ 2), ...
    (1 - sqrt(2) * W + W ^ 2) / (1 + sqrt(2) * W + W ^ 2)];
out('\n候选滤波器（2 阶 Butterworth lp %.0fHz, fs=500）:\n  b = [%.6g %.6g %.6g]\n  a = [1 %.6g %.6g]\n', ...
    fc, b1f, a1f - [0 0 0]);
% 相位滞后 @0.25Hz 与 @3.8Hz
for fx = [0.25, 3.8, 7.7]
    H = polyval(flip(b1f), exp(-1j * 2 * pi * fx / fs)) / polyval(flip(a1f), exp(-1j * 2 * pi * fx / fs));
    out('  f=%5.2fHz: |H|=%.3f  相位滞后=%.1f ms\n', fx, abs(H), -angle(H) / (2 * pi * fx) * 1000);
end

%% D. 综合建议
out('\n--- D. 综合建议（按预期 te 改善 × 置信度排序）---\n');
% B 图谱：若 pos_trim 均值随 θ 系统变化超过 ±150 duty → 位置前馈有肉
spread = max(mu_pt) - min(mu_pt);
out('1. 位置前馈：pos_trim(θ) 极差 %.0f duty（IQ 限幅 600）。', spread);
if spread > 150
    out('可支持一张 12 格前馈表（+dz LUT 修正），预期削掉位置相关直流误差。\n');
else
    out('各 θ 差异不大，位置前馈收益有限。\n');
end
out('2. 慢模态：主频均值 %.3fHz，阻尼 ζ 均值 %.3f（含坏 trim 段）。\n', ...
    mean(A(:, 1), 'omitnan'), mean(A(:, 2), 'omitnan'));
out('3. K4 权限：见 C 表——K4·θ̇_hf 注入若已接近 duty_hf_std，说明速度通道满权限，\n');
out('   换 6Hz 二阶滤波器（相位滞后 0.25Hz 处 <1ms）可再抬 K4 而不增抖动。\n');
disp('POS_DATA_ANALYSIS_DONE');
end
