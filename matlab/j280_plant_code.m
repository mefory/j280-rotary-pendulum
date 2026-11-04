function code=j280_plant_code(P)
% Embedded version of j280_plant_rhs: constants folded for Stateflow codegen.
J0=P.J_arm+P.m_rod*P.L_arm^2;Jp=P.J_rod+P.m_rod*P.lc_rod^2;C=P.m_rod*P.L_arm*P.lc_rod;
code=strjoin({
'function dx=plant(x,tau)'
sprintf('J0=%.17g; Jp=%.17g; C=%.17g;',J0,Jp,C)
sprintf('mgl=%.17g; bv=%.17g; bp=%.17g;',P.m_rod*P.g*P.lc_rod,P.b_vis,P.b_rod)
's=sin(x(2)); c=cos(x(2));'
'M11=J0+Jp*s*s; M12=C*c;'
'r1=tau-2*Jp*s*c*x(3)*x(4)+C*s*x(4)^2-bv*x(3);'
'r2=mgl*s+Jp*s*c*x(3)^2-bp*x(4);'
'd=M11*Jp-M12*M12;'
'dx=[x(3);x(4);(Jp*r1-M12*r2)/d;(-M12*r1+M11*r2)/d];'},newline);
end
