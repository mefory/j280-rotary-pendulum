function audit_physics()
[A,B,P]=j280_linear_model(); eps0=1e-6; An=zeros(4);Bn=zeros(4,1);
for k=1:4
    d=zeros(4,1);d(k)=eps0;
    An(:,k)=(j280_plant_rhs(d,0,P)-j280_plant_rhs(-d,0,P))/(2*eps0);
end
Bn=(j280_plant_rhs(zeros(4,1),eps0*P.Kt/P.R,P)-j280_plant_rhs(zeros(4,1),-eps0*P.Kt/P.R,P))/(2*eps0);
assert(norm(An-A,Inf)<1e-6 && norm(Bn-B,Inf)<1e-6,'Nonlinear/linear Jacobian mismatch');
rng(17);err=0;
for k=1:100
    x=[randn;2*pi*rand-pi;5*randn;5*randn];tau=.1*randn;
    f=j280_plant_rhs(x,tau,P);grad=zeros(1,4);
    for j=1:4
        d=zeros(4,1);d(j)=eps0;
        grad(j)=(energy(x+d,P)-energy(x-d,P))/(2*eps0);
    end
    balance=tau*x(3)-P.b_vis*x(3)^2-P.b_rod*x(4)^2;
    err=max(err,abs(grad*f-balance));
end
assert(err<1e-6,'Energy balance mismatch');
fprintf('PHYSICS PASS: analytic/numerical Jacobian, 100 energy balances, max error %.2g\n',err);
lqr_lag_check();
end
function e=energy(x,P)
J0=P.J_arm+P.m_rod*P.L_arm^2;Jp=P.J_rod+P.m_rod*P.lc_rod^2;C=P.m_rod*P.L_arm*P.lc_rod;
M=[J0+Jp*sin(x(2))^2 C*cos(x(2));C*cos(x(2)) Jp];
e=.5*x(3:4)'*M*x(3:4)+P.m_rod*P.g*P.lc_rod*cos(x(2));
end
