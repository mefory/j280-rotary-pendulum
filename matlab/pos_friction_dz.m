function pos_friction_dz()
% pos_friction_dz.m — dz 摩擦补偿审计（纯数据，无模型尺度依赖）
% E1: dz_added = duty − duty_pre_dz 随 |θ̇| 的实际注入曲线（速度自适应 REF 的实测形状）
% E2: 粘-滑突破事件：静止≥300ms 后 |θ̇| 升破 15°/s 的瞬间，
%     前 100ms LPF duty ≈ 突破需求 → 对比当时 dz_added = 补偿缺口
D = dir(fullfile(fileparts(mfilename('fullpath')), 'data', 'pos_*.csv'));
fid = fopen(fullfile(fileparts(mfilename('fullpath')), 'data', '_dz_audit.txt'), 'w');
assert(fid >= 0); cleanup = onCleanup(@()fclose(fid));
P = @(varargin) fprintf(fid, varargin{:});
out = @(varargin) {fprintf(1, varargin{:}); P(varargin{:})};

edges = [0 20 50 100 200 400 1000];       % °/s
NB = numel(edges) - 1;
s_dz = zeros(NB, 1); c_dz = zeros(NB, 1);
s_du = zeros(NB, 1); c_du = zeros(NB, 1);
brk_du = []; brk_dz = []; brk_th = [];

for i = 1:numel(D)
    T = readtable(fullfile(D(i).folder, D(i).name));
    dz = T.duty - T.duty_pre_dz;                   % 实际注入的摩擦补偿
    thd = T.theta_dot / 10;                        % °/s
    a = abs(thd);
    for k = 1:NB
        m = a >= edges(k) & a < edges(k+1);
        s_dz(k) = s_dz(k) + sum(abs(dz(m)));
        c_dz(k) = c_dz(k) + nnz(m);
        s_du(k) = s_du(k) + sum(abs(T.duty(m)));
        c_du(k) = c_du(k) + nnz(m);
    end
    % E2 粘→滑事件：|θ̇|<5°/s 持续 150ms 后 100ms 内升破 15°/s
    stick = a < 5;
    cnt = 0;                                       % 连续静止计数
    j = 1;
    n = numel(a);
    while j <= n
        if stick(j), cnt = cnt + 1; else
            if cnt >= 150 && j + 100 <= n          % ≥300ms
                w = a(j:min(j+100, n));
                if max(w) > 15
                    k1 = max(1, j - 50);
                    du = mean(T.duty(k1:j));       % 突破前 100ms LPF duty
                    dzv = mean(dz(k1:j));
                    brk_du(end+1) = abs(du);       %#ok<AGROW>
                    brk_dz(end+1) = abs(dzv);      %#ok<AGROW>
                    brk_th(end+1) = mod(T.theta_c01(j) / 100, 360); %#ok<AGROW>
                end
            end
            cnt = 0;
        end
        j = j + 1;
    end
end
out('=== E1: dz 实际注入随 |θ̇| （±均值）===\n%8s %9s %9s %10s\n', 'θ̇(°/s)', 'n', '|dz|', '|duty|');
for k = 1:NB
    out('%3d-%4d %9d %9.0f %10.0f\n', edges(k), edges(k+1), c_dz(k), ...
        s_dz(k) / max(c_dz(k), 1), s_du(k) / max(c_du(k), 1));
end
out('\n=== E2: 粘-滑突破（n=%d 事件）===\n', numel(brk_du));
if ~isempty(brk_du)
    out('突破前 |duty|:  中位 %.0f  P25 %.0f  P75 %.0f\n', ...
        median(brk_du), prc(brk_du, 25), prc(brk_du, 75));
    out('其中 |dz| 注入: 中位 %.0f  P25 %.0f  P75 %.0f\n', ...
        median(brk_dz), prc(brk_dz, 25), prc(brk_dz, 75));
    out('突破点 θ 分布（15° 格）: '); out(sprintf('%d ', ...
        histcounts(brk_th, 0:15:360)));
end
disp('POS_FRICTION_DZ_DONE');
end

function v = prc(x, p)
x = sort(x(:));
v = x(max(1, min(numel(x), round(p / 100 * numel(x)))));
end
