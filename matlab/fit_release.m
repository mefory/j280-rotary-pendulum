% fit_release.m — 用非线性植物 ODE 拟合释放衰减轨迹（2026-09-30 齿隙修复后重辨识）
%
% 数据：tmp/ident_data/release_*.jsonl（STOP 释放后的 α/θ 遥测，500Hz）
% 模型：j280_plant_rhs（Furuta，x=[θ臂, α摆(0=倒立), θ̇, α̇]，SI 单位）
%       摆杆侧扩展库仑摩擦：r2 -= tau_c*tanh(x(4)/0.3)
% 拟合：p = [J_rod, b_rod, tau_c, lc_rod]，fminsearch（基础 MATLAB）
%       初始状态取自轨迹 t=0.3s 的遥测（含差分速度）
% 输出：命令行汇总 + 直接更新 config/plant_de10_identified.json
function fit_release()
here = fileparts(mfilename('fullpath'));
files = dir(fullfile(here, '..', '..', 'tmp', 'ident_data', 'fit_input_*.csv'));
assert(~isempty(files), '找不到 fit_input_*.csv（先跑 tmp/export_fit_csv.py）');

P0 = j280_params();   % 几何/电机先验（m_rod/L_arm/J_arm/g）
ts_cell = {}; al_cell = {}; th_cell = {}; ctr_cell = {};
for k = 1:numel(files)
    M = readmatrix(fullfile(files(k).folder, files(k).name), 'NumHeaderLines', 1);
    T = M(:,1)'; A = M(:,2)'; TH = M(:,3)';
    TH = TH - TH(1);            % 臂角相对化（度）
    ts_cell{end+1} = T; al_cell{end+1} = A; th_cell{end+1} = TH; %#ok<AGROW>
    ctr_cell{end+1} = -180; %#ok<AGROW>  % CSV 已按悬挂点=0 导出，center 仅用于 sim 口径换算
    fprintf('%s: %d 样本 %.1fs\n', files(k).name, numel(A), T(end));
end

% 目标函数：p = [J_rod, b_rod, tau_c, lc_rod]
p0 = [0.000520658, 0.000361739, 0.02, 0.0675];   % 09-28 值 + 库仑初值
obj = @(p) trace_residual(p, ts_cell, al_cell, th_cell, ctr_cell, P0);
opts = optimset('MaxFunEvals', 3000, 'MaxIter', 1500, 'Display', 'iter');
[pfit, res] = fminsearch(obj, p0, opts);

fprintf('\n===== 拟合结果 =====\n');
fprintf('J_rod  = %.9f  (09-28: 0.000520658, 几何: 0.000213)\n', pfit(1));
fprintf('b_rod  = %.9f  (09-28: 0.000361739)\n', pfit(2));
fprintf('tau_c  = %.6f N·m（库仑，摆杆轴承+电位器）\n', pfit(3));
fprintf('lc_rod = %.6f m  (先验 0.0675)\n', pfit(4));
fprintf('残差（度²样本均值）= %.4f\n', res);

% 直接更新 JSON（motor_gain/b_vis 沿用 09-28）
cfg = fullfile(here, '..', 'config', 'plant_de10_identified.json');
prof = jsondecode(fileread(cfg));
old = prof.parameters;
prof.parameters.J_rod = pfit(1);
prof.parameters.b_rod = pfit(2);
prof.parameters.lc_rod = pfit(4);
prof.source = sprintf('DE10 release-decay ODE fit x%d, 2026-09-30 (backlash-fixed); motor_gain/b_vis kept from 09-28', numel(files));
fid = fopen(cfg, 'w'); fwrite(fid, jsonencode(prof, 'PrettyPrint', true)); fclose(fid);
fprintf('%s 已更新（J_rod %.6f→%.6f, b_rod %.6f→%.6f, lc %.4f→%.4f）\n', ...
    cfg, old.J_rod, pfit(1), old.b_rod, pfit(2), 0.0675, pfit(4));
end

function res = trace_residual(p, ts_cell, al_cell, th_cell, ctr_cell, P0)
% 坐标映射：模型 x(2)=摆角以倒立为 0（rad）；遥测 A 以悬挂点为 0（度），
% 悬挂点在遥测中的角度 = center（±180，解回绕多圈等价）→ x2 = radians(center + A)
P = P0;
P.J_rod = max(p(1), 1e-6);
P.b_rod = max(p(2), 1e-6);
P.lc_rod = min(max(p(4), 0.02), 0.13);
tau_c = max(p(3), 0);
% 可行域：质量矩阵行列式 d = M11*Jp - M12^2 必须恒正（最坏 cos=1）
C2 = (P.m_rod*P.L_arm*P.lc_rod)^2;
Jp = P.J_rod + P.m_rod*P.lc_rod^2;
if (P.J_arm + P.m_rod*P.L_arm^2 + Jp)*Jp <= C2*1.05
    res = 1e7; return;
end
res = 0;
for k = 1:numel(ts_cell)
    T = ts_cell{k}; A = al_cell{k}; TH = th_cell{k}; center = ctr_cell{k};
    % 初始状态 @ t=0.05s：刚释放，摆杆近倒立、速度小（良态）；0.15s 窗口差分
    i0 = find(T >= 0.05, 1);
    i1 = find(T >= T(i0) + 0.15, 1);
    im = find(T >= (T(i0)+T(i1))/2, 1);
    a0 = center + A(im);
    th0 = TH(im);
    ad0 = (A(i1) - A(i0)) / max(T(i1) - T(i0), 1e-3);
    td0 = (TH(i1) - TH(i0)) / max(T(i1) - T(i0), 1e-3);
    x = [th0*pi/180; a0*pi/180; td0*pi/180; ad0*pi/180];
    dt = 1e-3;
    t_end = min(T(end), 25);
    n = round(t_end / dt);
    simA = zeros(1, n); simT = zeros(1, n);
    for i = 1:n
        if i > 1
            x = x + rk4(x, dt, P, tau_c);
        end
        simA(i) = x(2)*180/pi - center;
        simT(i) = x(1)*180/pi;
        if abs(simA(i)) > 3600 || abs(simT(i)) > 36000
            res = 1e7; return;   % 发散护栏
        end
    end
    % 遥测采样点残差（前 25s 全部）
    nres = 0;
    for i = 1:numel(T)
        tt = T(i);
        if tt > t_end, break; end
        j = max(1, min(n, round(tt/dt)+1));
        res = res + (simA(j)-A(i))^2 + 0.25*(simT(j)-TH(i))^2;
        nres = nres + 1;
    end
    res = res / max(nres, 1);
end
res = res / max(numel(ts_cell), 1);
end

function dx = rk4(x, dt, P, tau_c)
k1 = rhs_p(x, P, tau_c);
k2 = rhs_p(x + dt/2*k1, P, tau_c);
k3 = rhs_p(x + dt/2*k2, P, tau_c);
k4 = rhs_p(x + dt*k3, P, tau_c);
dx = dt/6*(k1 + 2*k2 + 2*k3 + k4);
end

function dx = rhs_p(x, P, tau_c)
dx = j280_plant_rhs(x, 0, P);
dx(4) = dx(4) - tau_c*tanh(x(4)/0.3) / (P.J_rod + P.m_rod*P.lc_rod^2);
end

function y = unwrap_rad(x)
y = x;
for i = 2:numel(x)
    d = x(i) - x(i-1);
    d = d - 2*pi*round(d/(2*pi));
    y(i) = y(i-1) + d;
end
end
