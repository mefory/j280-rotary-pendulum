function k3_scale_scan()
% K3 放大扫描：位置刚度是当前短板（3.5° 误差只出 88 duty << 死区 350）
[A,B,P] = j280_linear_model();
kq = [21858 15748 2064 13166];
fid = fopen(fullfile(fileparts(mfilename('fullpath')),'..','logs','_k3scan.txt'),'w');
assert(fid>=0); c = onCleanup(@()fclose(fid));
fprintf(fid,'%6s %8s %8s %8s %8s %9s %8s  %s\n','scale','K1','K2','K3','K4','rho','eigmax','判定');
for s = [1.0 1.5 2.0 3.0 4.0 6.0]
    k = double(kq); k(3) = round(kq(3)*s);
    Kq = -k([3 1 4 2])/2^13/(P.duty_full/P.Vbus).*[18000 18000 1800 1800]/pi;
    em = max(real(eig(A-B*Kq)));
    E = expm([A B;zeros(1,5)]*P.Ts_ctrl); Ad=E(1:4,1:4); Bd=E(1:4,5);
    F = zeros(18);
    for j=1:18
        z=zeros(18,1); z(j)=1; x=z(1:4); ah=z(5:9); th=z(10:14);
        av=z(16)+(25*z(15)-z(16))/4; tv=z(17)+(25*(th(1)-th(5))-z(17))/4;
        u=(k/2^13)*[ah(1);av;th(1);tv]*P.Vbus/P.duty_full;
        F(:,j)=[Ad*x+Bd*z(18);x(2)*18000/pi;ah(1:4);x(1)*18000/pi;th(1:4);
                ah(1)-ah(5);av;tv;u];
    end
    rho = max(abs(eig(F)));
    v = 'OK'; if em>=0 || rho>=1, v='不稳'; elseif rho>0.999, v='裕量紧'; end
    fprintf(fid,'%6.2f %8d %8d %8d %8d %9.6f %8.3f  %s\n',s,k(1),k(2),k(3),k(4),rho,em,v);
end
fprintf(fid,'DONE\n');
end
