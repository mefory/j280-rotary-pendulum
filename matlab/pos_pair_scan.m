function pos_pair_scan()
% pos_pair_scan.m — 新辨识植物上的 POS 对（id11/id12）推导（2026-09-30）
% 思路：POS 模式 = 同一线性植物，臂环换成重位置权重的 LQR 解 K3/K4。
% 扫 q_theta_pos（位置权重）× q_tdot，取 s16 合规 + 量化闭环稳定的组合。
[A,B,P] = j280_linear_model(j280_params_de10_identified());
R = 1;
fid = fopen(fullfile(fileparts(mfilename('fullpath')),'..','logs','_pos_pair_scan.txt'),'w');
assert(fid>=0); cleanup = onCleanup(@()fclose(fid));
fprintf(fid,'%8s %6s %8s %8s %9s %7s %8s %s\n','q_theta','q_tdot','K3','K4','rho','eigmax','s16','判定');
fprintf(fid,'%s\n',repmat('-',1,80));
for qp = [0.1 0.3 1.0 3.0 10.0 30.0]
    for td = [0.01 0.02 0.05]
        Q = diag([qp, 2, 0.05, td]);
        % 状态序（j280_linear_model）：[θ臂, α摆, θ̇, α̇]——qp 加在臂位置上。
        % 注意 lqr_scan_identified 原表的 q_theta/q_tdot 标签是错位的（实际
        % 扫的是 θ̇/α̇ 权重），位置权重一直钉在 0.05。
        K = lqr(A,B,Q,R);
        k = -K([2 4 1 3])*(P.duty_full/P.Vbus).*pi./[18000 1800 18000 1800];
        kq = round(k*2^13); kq = kq(:)';
        K3 = kq(3); K4 = kq(4);
        % 按实机真实组合验收：K1/K2 = 已部署的新平衡增益，K3/K4 = 候选 POS 对
        kq(1) = 24205; kq(2) = 12019;
        [ok, rho, eigmax] = check_candidate(kq, A, B, P);
        s16 = 'OK'; if abs(K3) >= 32768 || abs(K4) >= 32768, s16 = '溢出'; end
        v = 'OK';
        if strcmp(s16,'溢出'), v = 's16溢出';
        elseif ~ok, v = '闭环不稳';
        elseif rho >= 1, v = '不稳';
        end
        fprintf(fid,'%8.1f %6.2f %8d %8d %9.6f %7.3f %8s %s\n', ...
            qp, td, K3, K4, rho, eigmax, s16, v);
    end
end
fprintf(fid,'DONE\n');
disp('POS_PAIR_DONE');
end

function [ok, rho, em] = check_candidate(kq, A, B, P)
% 18 状态量化采样环（k3_scale_scan 同款）：LQR 连续极点 + 定点/EMA/4ms 差分/采样
Kq = -kq([3 1 4 2])/2^13/(P.duty_full/P.Vbus).*[18000 18000 1800 1800]/pi;
em = max(real(eig(A - B*Kq)));
E = expm([A B;zeros(1,5)]*P.Ts_ctrl); Ad=E(1:4,1:4); Bd=E(1:4,5);
F = zeros(18);
for j=1:18
    z=zeros(18,1); z(j)=1; x=z(1:4); ah=z(5:9); th=z(10:14);
    av=z(16)+(25*z(15)-z(16))/4; tv=z(17)+(25*(th(1)-th(5))-z(17))/4;
    u=(kq/2^13)*[ah(1);av;th(1);tv]*P.Vbus/P.duty_full;
    F(:,j)=[Ad*x+Bd*z(18);x(2)*18000/pi;ah(1:4);x(1)*18000/pi;th(1:4);
            ah(1)-ah(5);av;tv;u];
end
rho = max(abs(eig(F)));
ok = (em < 0) && (rho < 1);
end
