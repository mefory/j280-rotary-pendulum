function audit_design()
[A,B,P] = j280_linear_model();
disp('Correct plant A:'); disp(A);
for q1 = [0.3 1 4 8 16 32]
    for q2 = [8 32 64]
        K = lqr(A,B,diag([q1 q2 .05 .05]),1);
        k = -K([2 4 1 3]).*(P.duty_full/P.Vbus)*pi./[18000 1800 18000 1800];
        fprintf('Q=[%g %g .05 .05] k=%s poles=%s\n',q1,q2,mat2str(k,5),mat2str(eig(A-B*K)',4));
    end
end
end
