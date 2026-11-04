function lqr_scan_identified()
% lqr_scan_identified.m — 在候选辨识植物上重整定 LQR（2026-09-28 定案课题）。
%
% 背景：R5 增益组 [17751,11613,206,7104] 在 WHEELTEC 先验模型上整定，真机
% 边缘稳定（捕获后存活 0.5~30s 漂移）。候选辨识模型（plant_de10_identified，
% 五条真机甩摆轨迹拟合）四参数显著偏离先验：J_rod ×2.4、b_rod ×3.6、
% motor_gain=0.663——先验增益在真机临界是模型错配的必然。
%
% 实机硬约束（2026-09-27/28 定案）：
%   k3 ≤ 400（BALANCE 臂位环：206 可活，≥700 移动中掉；定位由 POS 对负责）
%   k4 ≤ 8000（9000+ 即便限时增强也崩）
%   全部 |kq| < 32768（s16 Q3.13）；量化闭环 eigmax<0；采样环 ρ<0.995
% 输出 logs/_lqr_scan_identified.txt；选候选后跑 lqr_to_rtl 定稿。

[A,B,P] = j280_linear_model(j280_params_de10_identified());
R = 1;
q_theta = [0.01 0.02 0.03 0.05];
alpha_w_vec = [1 2 4 8];
theta_dot_w = [0.05 0.02 0.01];

fid = fopen(fullfile(fileparts(mfilename('fullpath')),'..','logs','_lqr_scan_identified.txt'),'w');
assert(fid>=0); cleanup = onCleanup(@()fclose(fid));

fmt = '%-7s %-6s %8s %8s %8s %8s %8s %9s %7s %8s %s\n';
fprintf(fid,fmt,'q_theta','q_tdot','K1','K2','K3','K4','K4/K3','rho','eigmax','约束','判定');
fprintf(fid,'%s\n',repmat('-',1,100));

for aw = alpha_w_vec
  for tw = q_theta
    for td = theta_dot_w
        Q = diag([0.05, aw, tw, td]);
        K = lqr(A,B,Q,R);
        k = -K([2 4 1 3])*(P.duty_full/P.Vbus).*pi./[18000 1800 18000 1800];
        kq = round(k*2^13); kq = kq(:)';
        [ok, rho, eigmax] = check_candidate(kq, A, B, P);
        cons = 'OK';
        if any(abs(kq) >= 32768), cons = 's16溢出'; end
        if abs(kq(3)) > 400 && strcmp(cons,'OK'), cons = 'k3>400'; end
        % 2026-09-30：k4≤8000 是齿隙期的实机限制，机械修复后放宽为标注不排除
        if abs(kq(4)) > 8000 && strcmp(cons,'OK'), cons = 'k4>8000(待验)'; end
        fmt_row = sprintf('aw=%-4g', aw);
        verdict = 'OK';
        if ~strcmp(cons,'OK') && ~strcmp(cons,'k4>8000(待验)')
            verdict = cons;
        elseif strcmp(cons,'OK') || strcmp(cons,'k4>8000(待验)')
            if ~ok, verdict = '闭环不稳'; end
            if ok && rho > 0.995, verdict = '裕量不足'; end
        end
        fprintf(fid,'%-7s %-6s %8s %8s %8s %8s %8s %9s %7s %8s %s\n', ...
            sprintf('%g',tw),num2str(td), ...
            num2str(kq(1)), num2str(kq(2)), num2str(kq(3)), num2str(kq(4)), ...
            sprintf('%.2f',abs(kq(4))/max(abs(kq(3)),1)), ...
            sprintf('%.6f',rho), sprintf('%.4f',eigmax), cons, verdict);
    end
  end
end
fprintf(fid,'\n基线 R5(先验模型) = [17751,11613,206,7104]；实机约束 k3≤400，k4>8000 为齿隙期待验证项\n');
fprintf(fid,'SCAN_DONE\n');
disp('SCAN_DONE');
end

function [ok, rho, eigmax] = check_candidate(kq, A, B, P)
% 复刻 lqr_lag_check 的 18 状态采样环模型（含 4ms 差分 + EMA/4 + 一拍作动延迟）
E = expm([A B;zeros(1,5)]*P.Ts_ctrl); Ad = E(1:4,1:4); Bd = E(1:4,5);
F = zeros(18);
for j = 1:18
    z = zeros(18,1); z(j) = 1;
    x = z(1:4); ah = z(5:9); th = z(10:14);
    av = z(16)+(25*z(15)-z(16))/4;
    tv = z(17)+(25*(th(1)-th(5))-z(17))/4;
    u = (kq/2^13)*[ah(1);av;th(1);tv]*P.Vbus/P.duty_full;
    F(:,j) = [Ad*x+Bd*z(18); x(2)*18000/pi; ah(1:4); x(1)*18000/pi; th(1:4);
              ah(1)-ah(5); av; tv; u];
end
rho = max(abs(eig(F)));
Kq = -kq([3 1 4 2])/2^13/(P.duty_full/P.Vbus).*[18000 18000 1800 1800]/pi;
eigmax = max(real(eig(A-B*Kq)));
ok = (eigmax < 0) && (rho < 1);
end
