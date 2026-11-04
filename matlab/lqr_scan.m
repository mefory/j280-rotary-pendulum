function lqr_scan()
% lqr_scan.m — 位置环重设计：扫描 θ（摆臂角）权重，看 K1..K4 与稳定裕量怎么变。
%
% 背景（2026-09-23 定位专项）：现役 Q = diag([0.03 8 0.05 0.05])，状态序 [θ α θ̇ α̇]。
% 摆杆角 α 权重 8，摆臂角 θ 权重只有 0.03（差 267 倍）→ K3 = 0.0251 duty/cdeg，
% 满 30° 误差只出 75 duty，而执行器死区是 350 → 位置环没有权限，定位实际靠
% 死区补偿抖动"爬"，方向不可控（实机摆臂来回摆、到不了目标）。
%
% 判据（本脚本内）：
%   ① 量化后闭环连续极点实部全负（整数增益下仍稳定）
%   ② 采样环谱半径 ρ < 1，且留裕量（lqr_lag_check 的 18 状态模型，含 4ms 差分/
%      EMA/采样保持/一拍作动延迟——它把 θ̇ 估计器的滞后也算进去了）
%   ③ 权限指标  |K3|×3000/8192 ≫ 死区 350
% 输出 rtl 写入不动；选出候选后再改 Q 跑 lqr_to_rtl.m。
[A,B,P] = j280_linear_model();
Q0 = diag([0.03 8 0.05 0.05]); R = 1;
theta_w = [0.03 0.1 0.3 1 3 10 30];
tdot_w  = [0.05 0.01];

fid = fopen(fullfile(fileparts(mfilename('fullpath')),'..','logs','_lqr_scan.txt'),'w');
assert(fid>=0); cleanup = onCleanup(@()fclose(fid));

fmt = '%-6s %-7s %8s %8s %8s %8s %8s %8s %8s %9s %7s\n';
fprintf(fid,fmt,'q_theta','q_tdot','K1','K2','K3','K4','K3*3000','K4/K3','rho','eigmax','判定');
fprintf(fid,'%s\n',repmat('-',1,95));

rows = 0;
for tw = theta_w
    for td = tdot_w
        Q = Q0; Q(1,1) = tw; Q(4,4) = td;
        K = lqr(A,B,Q,R);
        k = -K([2 4 1 3])*(P.duty_full/P.Vbus).*pi./[18000 1800 18000 1800];
        kq = round(k*2^13);
        kq = kq(:)';
        [ok, rho, eigmax] = check_candidate(kq);
        rows = rows + 1;
        verdict = 'OK';
        if any(abs(kq) >= 32768), verdict = 'Q3.13溢出'; end
        if ~ok && strcmp(verdict,'OK'), verdict = '闭环不稳'; end
        if ok && rho > 0.995, verdict = '裕量不足'; end
        fprintf(fid,fmt, num2str(tw),num2str(td), ...
            num2str(kq(1)),num2str(kq(2)),num2str(kq(3)),num2str(kq(4)), ...
            sprintf('%.0f',abs(kq(3))*3000/8192), sprintf('%.2f',abs(kq(4))/abs(kq(3))), ...
            sprintf('%.6f',rho), sprintf('%.4f',eigmax), verdict);
    end
end
fprintf(fid,'\n死区=350 duty；权限指标 K3*3000 = 30° 误差时的比例输出(duty)，需 ≫350\n');
fprintf(fid,'%.0f 行\n',rows);
fprintf(fid,'SCAN_DONE\n');
disp('SCAN_DONE');
end

function [ok, rho, eigmax] = check_candidate(kq)
% 复刻 lqr_lag_check 的 18 状态采样环模型（含 4ms 差分 + EMA/4 + 一拍作动延迟）
[A,B,P] = j280_linear_model();
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
