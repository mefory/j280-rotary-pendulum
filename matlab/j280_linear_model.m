function [A, B, P] = j280_linear_model(P)
% Exact upright Jacobian of the nonlinear plant; input is motor voltage.
if nargin < 1, P = j280_params(); end
motor_gain = 1;
if isfield(P, 'motor_gain'), motor_gain = P.motor_gain; end
M = [P.J_arm + P.m_rod*P.L_arm^2, P.m_rod*P.L_arm*P.lc_rod; ...
     P.m_rod*P.L_arm*P.lc_rod, P.J_rod + P.m_rod*P.lc_rod^2];
G = M \ [0; P.m_rod*P.g*P.lc_rod];
D = M \ diag([P.b_vis, P.b_rod]);
A = [zeros(2), eye(2); zeros(2,1), G, -D];
B = [0; 0; M \ [motor_gain*P.Kt/P.R; 0]];
end
