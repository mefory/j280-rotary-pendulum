// ===========================================================================
// j280_obs_gains.vh - 4-state steady-state Kalman observer gains (AUTO-GENERATED)
// Generated: obs_to_rtl(5) @ 2026-10-16 | Ts=1ms | fixed point Q12.20
// States: [theta(c01), alpha(c01), theta_dot(0.1deg/s), alpha_dot(0.1deg/s)]
// Internal scaling: states further x256 inside obs_core.v
// Update: x+ = Adu*x + Bdu*duty + Lu*(y - x(theta,alpha))   [control units]
// Theory 1-sigma: theta 35.64 c01 | theta_dot 18.45 | alpha 335.13 c01 | alpha_dot 284.36
// ===========================================================================
localparam signed [31:0] OBS_ADU_00 = 32'sd1048576;  // 1.000000
localparam signed [31:0] OBS_ADU_01 = -32'sd69;  // -0.000066
localparam signed [31:0] OBS_ADU_02 = 32'sd10447;  // 0.009963
localparam signed [31:0] OBS_ADU_03 = 32'sd1;  // 0.000000
localparam signed [31:0] OBS_ADU_10 = 32'sd0;  // 0.000000
localparam signed [31:0] OBS_ADU_11 = 32'sd1048753;  // 1.000169
localparam signed [31:0] OBS_ADU_12 = 32'sd67;  // 0.000064
localparam signed [31:0] OBS_ADU_13 = 32'sd10484;  // 0.009999
localparam signed [31:0] OBS_ADU_20 = 32'sd0;  // 0.000000
localparam signed [31:0] OBS_ADU_21 = -32'sd13790;  // -0.013151
localparam signed [31:0] OBS_ADU_22 = 32'sd1040892;  // 0.992672
localparam signed [31:0] OBS_ADU_23 = 32'sd80;  // 0.000076
localparam signed [31:0] OBS_ADU_30 = 32'sd0;  // 0.000000
localparam signed [31:0] OBS_ADU_31 = 32'sd35327;  // 0.033690
localparam signed [31:0] OBS_ADU_32 = 32'sd13318;  // 0.012701
localparam signed [31:0] OBS_ADU_33 = 32'sd1048372;  // 0.999805
localparam signed [31:0] OBS_BDU_00 = 32'sd124;  // 0.000118
localparam signed [31:0] OBS_BDU_10 = -32'sd215;  // -0.000205
localparam signed [31:0] OBS_BDU_20 = 32'sd24748;  // 0.023602
localparam signed [31:0] OBS_BDU_30 = -32'sd42895;  // -0.040908
localparam signed [31:0] OBS_LU_00 = 32'sd122481;  // 0.116807
localparam signed [31:0] OBS_LU_01 = 32'sd18025;  // 0.017190
localparam signed [31:0] OBS_LU_10 = 32'sd2996;  // 0.002857
localparam signed [31:0] OBS_LU_11 = 32'sd197443;  // 0.188297
localparam signed [31:0] OBS_LU_20 = 32'sd763064;  // 0.727714
localparam signed [31:0] OBS_LU_21 = 32'sd131491;  // 0.125399
localparam signed [31:0] OBS_LU_30 = 32'sd83146;  // 0.079294
localparam signed [31:0] OBS_LU_31 = 32'sd2055431;  // 1.960212
function signed [31:0] obs_coef(input [2:0] i, input [2:0] j);
    case ({{i,j}})
    7'h00: obs_coef = OBS_ADU_00;  // 1.000000
    7'h01: obs_coef = OBS_ADU_01;  // -0.000066
    7'h02: obs_coef = OBS_ADU_02;  // 0.009963
    7'h03: obs_coef = OBS_ADU_03;  // 0.000000
    7'h04: obs_coef = OBS_BDU_00;  // 0.000118
    7'h05: obs_coef = OBS_LU_00;   // 0.116807
    7'h06: obs_coef = OBS_LU_01;   // 0.017190
    7'h08: obs_coef = OBS_ADU_10;  // 0.000000
    7'h09: obs_coef = OBS_ADU_11;  // 1.000169
    7'h0a: obs_coef = OBS_ADU_12;  // 0.000064
    7'h0b: obs_coef = OBS_ADU_13;  // 0.009999
    7'h0c: obs_coef = OBS_BDU_10;  // -0.000205
    7'h0d: obs_coef = OBS_LU_10;   // 0.002857
    7'h0e: obs_coef = OBS_LU_11;   // 0.188297
    7'h10: obs_coef = OBS_ADU_20;  // 0.000000
    7'h11: obs_coef = OBS_ADU_21;  // -0.013151
    7'h12: obs_coef = OBS_ADU_22;  // 0.992672
    7'h13: obs_coef = OBS_ADU_23;  // 0.000076
    7'h14: obs_coef = OBS_BDU_20;  // 0.023602
    7'h15: obs_coef = OBS_LU_20;   // 0.727714
    7'h16: obs_coef = OBS_LU_21;   // 0.125399
    7'h18: obs_coef = OBS_ADU_30;  // 0.000000
    7'h19: obs_coef = OBS_ADU_31;  // 0.033690
    7'h1a: obs_coef = OBS_ADU_32;  // 0.012701
    7'h1b: obs_coef = OBS_ADU_33;  // 0.999805
    7'h1c: obs_coef = OBS_BDU_30;  // -0.040908
    7'h1d: obs_coef = OBS_LU_30;   // 0.079294
    7'h1e: obs_coef = OBS_LU_31;   // 1.960212
    default: obs_coef = 32'sd0;
    endcase
endfunction
