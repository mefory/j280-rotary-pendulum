function dx = j280_plant_rhs(x, tau, P)
% Furuta model, alpha=0 upright. Rod axial inertia neglected.
% J_rod is about its centre of mass; parallel-axis term is added once.
J0 = P.J_arm + P.m_rod*P.L_arm^2;
Jp = P.J_rod + P.m_rod*P.lc_rod^2;
C = P.m_rod*P.L_arm*P.lc_rod;
s=sin(x(2)); c=cos(x(2));
M11=J0+Jp*s*s; M12=C*c;
r1=tau-2*Jp*s*c*x(3)*x(4)+C*s*x(4)^2-P.b_vis*x(3);
r2=P.m_rod*P.g*P.lc_rod*s+Jp*s*c*x(3)^2-P.b_rod*x(4);
d=M11*Jp-M12*M12;
dx=[x(3);x(4);(Jp*r1-M12*r2)/d;(-M12*r1+M11*r2)/d];
end
