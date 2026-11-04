`timescale 1ns/1ps
// Direct mode_fsm timeout check: force SWING, run 46k ticks, watch mode transitions.
module tb_swing_timeout_check;
    reg clk = 0, rst_n = 0, tick = 0;
    reg key_start = 0, key_mode = 0, key_stop = 0, key_back_dummy = 0;
    reg fault = 0;
    reg signed [15:0] alpha = 0, alpha_dot = 0, theta_dot = 0;
    reg arm_stalled = 0;
    wire [2:0] mode;
    wire auto_pos_unused;

    // 211437-era configuration (no AUTO_POS / CATCH_TO_POS / key_back)
    mode_fsm #(
        .CATCH_TD_LIM(16'sd3500),
        .CATCH_AD_LIM(16'sd4000),
        .CATCH_DWELL_MS(6'd8),
        .CATCH_AD_MIN(16'sd200),
        .CATCH_DEAD_BAND(16'sd500),
        .CATCH_DEAD_AD(16'sd300),
        .FALL_DWELL_MS(6'd30),
        .AUTO_RESWING_EN(1)
    ) dut (
        .clk(clk), .rst_n(rst_n), .tick(tick),
        .key_start(key_start), .key_mode(key_mode), .key_stop(key_stop),
        .fault(fault),
        .alpha(alpha), .alpha_dot(alpha_dot), .theta_dot(theta_dot),
        .arm_stalled(arm_stalled),
        .catch_ang(16'sd1500), .catch_ang_fall(16'sd9000),
        .mode(mode)
    );

    always #5 clk = ~clk;
    integer ms = 0;
    integer safe_entries = 0;
    always #1000 begin tick = 1; ms = ms + 1; #20; tick = 0; end  // 高20ns确保跨一个posedge clk

    initial begin
        #100; rst_n = 1;
        // SAFE -> IDLE -> SWING: hold key_start high across several ticks
        // (SAFE: key_start->IDLE on one tick; IDLE: key_start->SWING on next)
        repeat(2) @(posedge tick);
        key_start = 1;
        repeat(6) @(posedge tick);
        key_start = 0;
        repeat(2) @(posedge tick);
        $display("after start: mode=%0d (expect SWING=1)", mode);
        // swing forever with large arm speed and alpha outside capture window
        alpha = 16'sd17000; alpha_dot = 16'sd1000; theta_dot = 16'sd1000;
        repeat(46000) begin
            @(posedge tick);
            if (ms == 1000)  $display("t=1s  mode=%0d", mode);
            if (ms == 10000) $display("t=10s mode=%0d", mode);
            if (ms == 39999) $display("t=40s mode=%0d (expect SAFE=4)", mode);
            if (ms == 40002) begin
                $display("t=40s+ mode=%0d", mode);
                if (mode !== 4) begin
                    $display("TIMEOUT NOT TRIGGERED -- BUG CONFIRMED");
                    $finish;
                end
            end
        end
        $display("DONE mode=%0d", mode);
        $finish;
    end
endmodule
