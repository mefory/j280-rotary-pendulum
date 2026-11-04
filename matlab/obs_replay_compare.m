function obs_replay_compare(logfile)
%% obs_replay_compare — 观测器 vs 现行差分链 离线回放对比（阶段 0）
%% 双编码器方案 experiment/mt6835-gmr-observer，2026-10-16。
%% 用已存实机遥测（alpha/theta 量测 + RTL 差分链的 alpha_dot/theta_dot）回放
%% 4 状态稳态 Kalman 观测器，扫描过程噪声 qa 对比噪声/延迟；并按新传感
%% （MT6835 21 位 + GMR 40900 计数）的量测噪声算理论估计精度（稳态方差）。
%% 零工具箱依赖：Riccati 递推自实现（每步对称化保数值稳定），离散化用 expm。
%%
%% 用法：matlab -batch "cd(<本目录>); obs_replay_compare('<jsonl 路径>')"
%%       省略参数时默认用上位机日志库最新一份。

if nargin < 1 || isempty(logfile)
    d = fullfile(fileparts(mfilename('fullpath')),'..','logs');  % 遥测日志目录
    f = dir(fullfile(d, '2*.jsonl'));
    [~, ix] = max([f.datenum]);
    logfile = fullfile(d, f(ix).name);
end
fprintf('== 观测器离线回放 ==\n日志: %s\n', logfile);

%% ---- 1. 读遥测 ----
txt = fileread(logfile);
lines = strsplit(txt, {'\n', sprintf('\n')});
lines = lines(~cellfun(@isempty, lines));
n0 = numel(lines);
tck = zeros(n0,1); mode = zeros(n0,1); flt = zeros(n0,1);
al = zeros(n0,1); ald = zeros(n0,1); th = zeros(n0,1); thd = zeros(n0,1);
du = zeros(n0,1); ren = zeros(n0,1);
for k = 1:n0
    s = jsondecode(lines{k});
    if ~isfield(s,'sample'), continue; end
    s = s.sample;
    tck(k)=s.tick; mode(k)=s.mode; flt(k)=s.fault;
    al(k)=s.alpha; ald(k)=s.alpha_dot; th(k)=s.theta; thd(k)=s.theta_dot;
    du(k)=s.duty; ren(k)=s.run_en;
end
ok = tck>0; tck=tck(ok); mode=mode(ok); flt=flt(ok); al=al(ok); ald=ald(ok);
th=th(ok); thd=thd(ok); du=du(ok); ren=ren(ok);

%% ---- 2. 选分析段：优先最长 BALANCE(=2) 段 ----
% 注：不用 |α| 门限截段（平衡初期 ±30° 摆动是真实瞬态，截掉会把段打碎）；
% 线性观测器在 ±30° 内近似成立。
seg = pick_longest(mode==2 & flt==0);
src = 'BALANCE';
if numel(seg) < 250                       % <0.5s：退而求其次 run_en 段
    seg = pick_longest(ren==1 & flt==0); src = 'run_en';
end
if numel(seg) < 250
    seg = 1:numel(tck); src = '整段';
end
t = tck(seg)*1e-3; t = t - t(1);
Ts = median(diff(t));
fprintf('分析段: %s，%d 拍，Ts=%.1fms，时长 %.1fs\n', src, numel(seg), Ts*1e3, t(end));

% 物理单位：0.01°→rad，0.1°/s→rad/s，duty→电压
d2r = pi/180;
alr  = al(seg)*1e-2*d2r;   thr  = th(seg)*1e-2*d2r;
ald_r = ald(seg)*0.1*d2r;  thd_r = thd(seg)*0.1*d2r;
u = du(seg)/10000 * 12.0;

%% ---- 3. 模型 + 离散化 ----
P = j280_params(); [A, B] = j280_linear_model(P);
n = 4;
Mexp = expm([A B; zeros(1,n+1)]*Ts);
Ad = Mexp(1:n,1:n); Bd = Mexp(1:n,n+1);
C  = [1 0 0 0; 0 1 0 0];
Gd = [zeros(2,2); eye(2)];                % 过程噪声进两路加速度

% 量测噪声：旧传感（电位器+13线霍尔）——数据高通估计
w = max(5, round(0.04/Ts));               % 40ms 均值窗 ≈ 4Hz 高通
sig_al = std(hp(alr, w));  sig_th = std(hp(thr, w));
fprintf('旧量测噪声估计(含>4Hz真运动): σ_α=%.4f° σ_θ=%.4f°\n', sig_al/d2r, sig_th/d2r);
R_old = diag([sig_th sig_al].^2);

%% ---- 4. qa 扫描回放：观测器 vs RTL 差分链 ----
sa_r = std(hp(ald_r,w));  st_r = std(hp(thd_r,w));
fprintf('\n---- 同一量测下：观测器（qa 扫描）vs RTL 差分链 ----\n');
fprintf('RTL 差分链基准: α̇噪声 %.2f°/s，θ̇噪声 %.2f°/s\n', sa_r/d2r, st_r/d2r);
fprintf('%6s | %10s %6s | %10s %6s | %s\n', 'qa(rad/s²)', 'α̇噪声', '比值', 'θ̇噪声', '比值', 'α̇滞后');
for qa = [0.3 1 3 10 30]
    % 白噪声加速度 PSD=qa² 离散化：每步速度方差 = qa²·Ts（勿漏 Ts，
    % 否则等效放大 1/sqrt(Ts)≈22 倍，不稳极点上稳态方差会发散）
    [L, ~] = kalman_sse(Ad, Gd, C, diag([qa qa].^2)*Ts, R_old);
    xh = [thr(1); alr(1); 0; 0];
    Nh = numel(t); adot_o = zeros(Nh,1); tdot_o = zeros(Nh,1);
    for k = 1:Nh
        xh = Ad*xh + Bd*u(k);
        xh = xh + L*([thr(k); alr(k)] - C*xh);
        tdot_o(k) = xh(3); adot_o(k) = xh(4);
    end
    sa_o = std(hp(adot_o,w));  st_o = std(hp(tdot_o,w));
    [~, lg] = lag_of(hp(adot_o,w), hp(ald_r,w), Ts);
    fprintf('%6.1f | %8.2f°/s %6.2f | %8.2f°/s %6.2f | %+0.0f ms\n', ...
            qa, sa_o/d2r, sa_o/sa_r, st_o/d2r, st_o/st_r, lg*1e3);
end

%% ---- 5. 稳态理论 1σ（qa 扫描；qa 是物理扰动强度假设，非调参旋钮） ----
fprintf('\n---- 稳态理论 1σ（新传感 R：GMR 0.0088° + MT6835 噪声底 0.005°） ----\n');
R_new = diag([(0.0088*d2r)^2, (0.005*d2r)^2]);
fprintf('%8s | %9s %9s %9s %9s\n', 'qa(rad/s²)', 'θ(°)', 'θ̇(°/s)', 'α(°)', 'α̇(°/s)');
for qa = [0.1 0.3 1 3]
    [~, Xn] = kalman_sse(Ad, Gd, C, diag([qa qa].^2)*Ts, R_new);
    sn = sqrt(diag(Xn)) ./ [d2r; d2r; d2r; d2r];
    fprintf('%8.1f | %9.4f %9.3f %9.4f %9.3f\n', qa, sn);
end
fprintf('注: qa=物理扰动强度（摩擦/未建模项），实机标定后定档；旧传感的理论行因\n');
fprintf('    σ 估计含 >4Hz 真运动而偏悲观，回放表才是旧传感的实证结论。\n');
fprintf('结论要点: 观测器收益随量测噪声降低而放大；RTL 化时 qa 取回放噪声最平的档位。\n');
end

%% =============== 子函数 ===============
function y = hp(x, w)
    y = x - smoothdata(x, 'movmean', w);
end

function [L, X] = kalman_sse(A, G, C, Q, R)
    % 热启动：位置误差~量测噪声、速度误差~0.01 rad/s 量级（单位 rad）
    X = diag([sqrt(R(1,1)); sqrt(R(2,2)); 0.01; 0.01]).^2;
    for it = 1:100000
        Pp = A*X*A' + G*Q*G';
        K = (Pp*C') / (C*Pp*C' + R);
        Xn = (eye(size(A,1)) - K*C)*Pp;
        Xn = (Xn + Xn')/2;          % 对称化：小 R 时非对称漂移会发散
        if norm(Xn - X, 'fro') < 1e-12*max(norm(X,'fro'),1), X = Xn; break; end
        X = Xn;
    end
    Pp = A*X*A' + G*Q*G';
    L = (Pp*C') / (C*Pp*C' + R);
end

function s = pick_longest(mask)
    d = diff([0; mask; 0]);
    a = find(d==1); b = find(d==-1)-1;
    if isempty(a), s = []; return; end
    [~, ix] = max(b-a);
    s = a(ix):b(ix);
end

function [pk, lg] = lag_of(x, y, Ts)
% 手写归一化互相关（±maxlag 拍），零工具箱依赖
    maxlag = 50;
    x = x - mean(x); y = y - mean(y);
    nx = sqrt(sum(x.^2)*sum(y.^2));
    best = -inf; bl = 0;
    for l = -maxlag:maxlag
        if l >= 0, c = sum(x(1:end-l).*y(1+l:end));
        else,     c = sum(x(1-l:end).*y(1:end+l));
        end
        c = c / max(nx, eps);
        if c > best, best = c; bl = l; end
    end
    pk = best; lg = bl*Ts;
end
