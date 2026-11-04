function obs_to_rtl(qa)
%% obs_to_rtl — 4 状态稳态 Kalman 观测器增益导出（MATLAB → rtl/j280_obs_gains.vh）
%% 双编码器方案 experiment/mt6835-gmr-observer，2026-10-16。
%% 流程：倒立点线性模型(SI) → 单位变换到控制口径（θ/α=0.01°，θ̇/α̇=0.1°/s，
%% 输入=duty ±10000）→ 1ms 精确 ZOH 离散化 → 稳态 Kalman（Riccati 递推，
%% 与 obs_replay_compare 同源实现）→ Q12.20 量化 → 生成 Verilog 头文件
%% （系数 localparam + obs_coef(i,j) 取值函数，obs_core.v include 使用）。
%%
%% 用法：matlab -batch "cd(<matlab目录>); obs_to_rtl"        % qa 默认 5
%%       matlab -batch "obs_to_rtl(10)"                       % 指定过程噪声

if nargin < 1, qa = 5; end               % rad/s² 过程噪声（回放甜点 3~10）
Ts = 1e-3;                               % 观测器节拍 = 1ms 控制拍
QF = 20;                                 % 增益定点 Q12.20

%% ---- 模型（SI）与单位变换 ----
% 注意：j280_linear_model 状态序为 [θ, α, θ̇, α̇]（x1'=x3, x2'=x4），
% 单位变换与 RTL 接线必须沿用此序，勿按 [θ, θ̇, α, α̇] 惯例重排。
P = j280_params(); [A, B] = j280_linear_model(P);
cpr  = 180/pi*100;                       % c01 / rad
dps  = 180/pi*10;                        % 0.1°/s / (rad/s)
S    = diag([cpr, cpr, dps, dps]);       % SI 状态 → 控制口径状态（模型原生序）
Sy   = diag([cpr, cpr]);                 % SI 量测 → c01 量测

Mexp = expm([A B; zeros(1,5)]*Ts);
Ad = Mexp(1:4,1:4);  Bd = Mexp(1:4,5);
Adu = S*Ad/S;                            % 离散 A（控制口径）
Bdu = S*Bd*12/10000;                     % 每 duty 计数的输入增益
C  = [1 0 0 0; 0 1 0 0];
Gd = [zeros(2,2); eye(2)];

%% ---- 稳态 Kalman（SI 域求解，R 换算到 rad²；Lu 再变换回控制口径） ----
% 20261013_154022 高通估计：σ_θ=1.0427°、σ_α=0.4251° → c01（×100）
sig_th = 104.27;                         % c01
sig_al = 42.51;                          % c01
R  = diag(([sig_th sig_al]/cpr).^2);     % SI 域量测方差（rad²）
Qn = diag([qa qa].^2)*Ts;                % 白噪声 PSD 离散化：qa²·Ts
[L, Xss] = kalman_sse(Ad, Gd, C, Qn, R);
Lu = S*L/Sy;                             % 控制口径 innovation 增益
Xss_u = S*Xss*S;                         % 理论 1σ 换到控制口径显示
% 自检：闭环极点模须全 <1，DARE 残差须 ≈0（防伪收敛）
ev = sort(abs(eig(Ad - L*C)));
Pp = Ad*Xss*Ad' + Gd*Qn*Gd';
K = (Pp*C')/(C*Pp*C' + R);
dare_res = norm(Xss - (eye(4)-K*C)*Pp, 'fro');
fprintf('自检: 闭环极点模 [%.4f %.4f %.4f %.4f]（须全<1），DARE 残差 %.2e\n', ev, dare_res);
if ev(end) >= 1 || dare_res > 1e-6
    error('观测器 Riccati 未正确收敛，拒绝生成');
end

%% ---- 范围断言 + Q12.20 量化 ----
allc = [Adu(:); Bdu(:); Lu(:)];
if max(abs(allc)) >= 2^11
    error('增益超 Q12.20 范围（|max|=%.2f），需改格式', max(abs(allc)));
end
names = {'Adu','Bdu','Lu'};
vals  = {Adu, Bdu, Lu};
fprintf('增益范围: ');
for k = 1:3, fprintf('%s max|%.4f|  ', names{k}, max(abs(vals{k}(:)))); end
fprintf('\n理论 1σ（控制口径）: θ %.2f c01, α %.2f c01, θ̇ %.2f (0.1°/s), α̇ %.2f (0.1°/s)\n', ...
        sqrt(diag(Xss_u)));

%% ---- 生成 Verilog 头（注释 ASCII-only：Gowin/iverilog 编码安全） ----
outdir = fullfile(fileparts(mfilename('fullpath')), '..', 'rtl');
fid = fopen(fullfile(outdir, 'j280_obs_gains.vh'), 'w');
fprintf(fid, '// ===========================================================================\n');
fprintf(fid, '// j280_obs_gains.vh - 4-state steady-state Kalman observer gains (AUTO-GENERATED)\n');
fprintf(fid, '// Generated: obs_to_rtl(%g) @ 2026-10-16 | Ts=1ms | fixed point Q12.20\n', qa);
fprintf(fid, '// States: [theta(c01), alpha(c01), theta_dot(0.1deg/s), alpha_dot(0.1deg/s)]\n');
fprintf(fid, '// Internal scaling: states further x256 inside obs_core.v\n');
fprintf(fid, '// Update: x+ = Adu*x + Bdu*duty + Lu*(y - x(theta,alpha))   [control units]\n');
fprintf(fid, '// Theory 1-sigma: theta %.2f c01 | theta_dot %.2f | alpha %.2f c01 | alpha_dot %.2f\n', ...
        sqrt(diag(Xss_u)));
fprintf(fid, '// ===========================================================================\n');
emit_lp(fid, 'OBS_ADU', round(Adu*2^QF), 4, 4, Adu);
emit_lp(fid, 'OBS_BDU', round(Bdu*2^QF), 4, 1, Bdu);
emit_lp(fid, 'OBS_LU',  round(Lu *2^QF), 4, 2, Lu);
% obs_coef(i,j)：j=0..3 状态项；j=4 输入项；j=5/6 innovation θ/α 项
fprintf(fid, 'function signed [31:0] obs_coef(input [2:0] i, input [2:0] j);\n');
fprintf(fid, '    case ({{i,j}})\n');
for i = 1:4
    for j = 1:4
        fprintf(fid, '    7''h%02x: obs_coef = OBS_ADU_%d%d;  // %.6f\n', ...
                (i-1)*8+(j-1), i-1, j-1, Adu(i,j));
    end
    fprintf(fid, '    7''h%02x: obs_coef = OBS_BDU_%d0;  // %.6f\n', (i-1)*8+4, i-1, Bdu(i));
    fprintf(fid, '    7''h%02x: obs_coef = OBS_LU_%d0;   // %.6f\n', (i-1)*8+5, i-1, Lu(i,1));
    fprintf(fid, '    7''h%02x: obs_coef = OBS_LU_%d1;   // %.6f\n', (i-1)*8+6, i-1, Lu(i,2));
end
fprintf(fid, '    default: obs_coef = 32''sd0;\n');
fprintf(fid, '    endcase\n');
fprintf(fid, 'endfunction\n');
fclose(fid);
fprintf('已生成 rtl/j280_obs_gains.vh（qa=%g，R=旧传感回放估计）\n', qa);
end

function emit_lp(fid, name, qv, r, c, fv)
for i = 1:r
    for j = 1:c
        v = qv(i,j);
        if v < 0
            fprintf(fid, 'localparam signed [31:0] %s_%d%d = -32''sd%d;  // %.6f\n', ...
                    name, i-1, j-1, -v, fv(i,j));
        else
            fprintf(fid, 'localparam signed [31:0] %s_%d%d = 32''sd%d;  // %.6f\n', ...
                    name, i-1, j-1, v, fv(i,j));
        end
    end
end
end

function [L, X] = kalman_sse(A, G, C, Q, R)
    % 与 obs_replay_compare.kalman_sse 同源：Riccati 递推 + 对称化
    X = diag([sqrt(R(1,1)); sqrt(R(2,2)); 0.01; 0.01]).^2;
    for it = 1:100000
        Pp = A*X*A' + G*Q*G';
        K = (Pp*C') / (C*Pp*C' + R);
        Xn = (eye(size(A,1)) - K*C)*Pp;
        Xn = (Xn + Xn')/2;
        if norm(Xn - X, 'fro') < 1e-12*max(norm(X,'fro'),1), X = Xn; break; end
        X = Xn;
    end
    Pp = A*X*A' + G*Q*G';
    L = (Pp*C') / (C*Pp*C' + R);
end
