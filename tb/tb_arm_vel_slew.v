// Dynamic slew contract: every published speed changes by at most LIM per tick.
// Abrupt acceleration/reversal exposes using the previous raw target as state.
`timescale 1ns / 1ps
module tb_arm_vel_slew;
    localparam integer LIM = 1000;
    reg clk = 0, rst_n = 0, tick = 0;
    reg signed [31:0] pos = 0;
    wire signed [15:0] speed;
    wire signed [15:0] speed_fast;
    integer previous = 0, change, errors = 0, violations = 0;
    integer previous_fast = 0, change_fast, violations_fast = 0;
    integer largest_change = 0, checked = 0;
    always #10 clk = ~clk;
    arm_vel #(.USE_T(1), .SLEW_LIM(LIM)) dut (
        .clk(clk), .rst_n(rst_n), .tick(tick), .pos(pos),
        .theta_c01(), .theta_dot(speed), .arm_stalled()
    );
    arm_vel #(.USE_T(1), .SLEW_LIM(LIM), .PIPELINE_ON_TICK(0)) dut_fast (
        .clk(clk), .rst_n(rst_n), .tick(tick), .pos(pos),
        .theta_c01(), .theta_dot(speed_fast), .arm_stalled()
    );

    // One accelerated control tick leaves more than 28 clocks for T division.
    task step(input integer count_delta);
        begin
            @(negedge clk); pos = pos + count_delta;
            repeat (60) @(negedge clk);
            tick = 1;
            @(posedge clk); #1;
            change = $signed(speed) - previous;
            if (change < 0) change = -change;
            if (change > largest_change) largest_change = change;
            if (change > LIM) begin
                if (violations < 5)
                    $display("FAIL slew: previous=%0d speed=%0d change=%0d", previous, speed, change);
                violations = violations + 1;
            end
            previous = $signed(speed);
            change_fast = $signed(speed_fast) - previous_fast;
            if (change_fast < 0) change_fast = -change_fast;
            if (change_fast > LIM) begin
                $display("FAIL fast slew: previous=%0d speed=%0d change=%0d", previous_fast, speed_fast, change_fast);
                violations_fast = violations_fast + 1;
            end
            previous_fast = $signed(speed_fast);
            checked = checked + 1;
            @(negedge clk); tick = 0;
            repeat (40) @(negedge clk);
        end
    endtask

    task hold(input integer count_delta, input integer ticks);
        integer i;
        begin
            for (i = 0; i < ticks; i = i + 1) step(count_delta);
        end
    endtask

    task expect_speed(input integer value, input integer tolerance);
        begin
            if ($signed(speed) < value-tolerance || $signed(speed) > value+tolerance) begin
                $display("FAIL tracking: expected=%0d tolerance=%0d speed=%0d", value, tolerance, speed);
                errors = errors + 1;
            end
            if ($signed(speed_fast) < value-tolerance || $signed(speed_fast) > value+tolerance) begin
                $display("FAIL fast tracking: expected=%0d speed=%0d", value, speed_fast);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        repeat (5) @(negedge clk); rst_n = 1;
        hold(0, 330); expect_speed(0, 1);
        hold(8, 100); expect_speed(27696, 20);
        hold(-8, 120); expect_speed(-27696, 20);
        hold(0, 360); expect_speed(0, 1);
        hold(1, 60); expect_speed(3462, 20);
        hold(-1, 60); expect_speed(-3462, 20);
        hold(0, 360); expect_speed(0, 1);
        if (violations != 0 || violations_fast != 0) errors = errors + 1;
        $display("checked=%0d violations=%0d largest_change=%0d", checked, violations, largest_change);
        $display("fast_pipeline_violations=%0d", violations_fast);
        if (errors == 0) $display("tb_arm_vel_slew PASS");
        else $fatal(1, "tb_arm_vel_slew FAIL errors=%0d", errors);
        $finish;
    end
    initial begin
        #10_000_000; $fatal(1, "TIMEOUT");
    end
endmodule
