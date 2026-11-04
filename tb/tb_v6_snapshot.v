`timescale 1ns/1ps
// v6 遥测对拍：debug_link 直连，CHECK 命令触发遥测快照，抓 send_payload 位宽。
module tb_v6_snapshot;
    reg clk = 0, rst_n = 0, tick = 0;
    reg rx_valid = 0;
    reg [7:0] rx_byte = 0;
    wire tx_start; wire [7:0] tx_byte; wire tx_busy;
    reg [2:0] mode = 3'd3;   // POS
    reg fault = 0, otr_fault = 0, run_en = 1, calib_done = 1;
    reg signed [15:0] alpha = 0, alpha_dot = 0, theta_dot = 0, duty = 0, offset_c01 = 0;
    reg signed [31:0] theta = 0, reference = 0, target = 0, encoder = 0;
    reg [15:0] adc = 0, diag_adc = 0;
    reg signed [31:0] diag_encoder = 0;
    reg signed [15:0] diag_pos_trim = 111, diag_alpha_pre = 222, diag_k3_eff = 333,
                      diag_k4_eff = 444, diag_duty_pre_dz = 555, diag_theta_e = 666,
                      diag_term_k1 = 0, diag_term_k2 = 0, diag_term_k3 = 0, diag_term_k4 = 0;
    reg diag_obs_active = 0;
    reg signed [31:0] diag_theta_raw = 32'sd777;
    reg signed [15:0] diag_alpha_raw = 888, diag_theta_dot_raw = 999, diag_alpha_dot_raw = 1111;
    reg signed [15:0] diag_k1_eff = 5, diag_k2_eff = 6, diag_pos_i_trim = 7;
    reg signed [15:0] diag_pos_bias = -325;
    reg [15:0] diag_move_blend = 1024, diag_settle_ms = 1500;
    reg [3:0] diag_dz_idx = 9;
    reg local_stop = 0, local_start = 0, arm_stalled = 1;
    wire remote_start, remote_stop, remote_clear, remote_mode, remote_calib;
    wire move_pending; wire signed [31:0] move_delta; wire config_lock;
    wire [31:0] revision = 0;
    wire [31:0] rx_errors;
    reg [15:0] mt6_frames = 0, mt6_good = 0, mt6_crc_err = 0, mt6_mg_err = 0;
    reg [2:0] mt6_mg_code = 0; reg [13:0] mt6_angle_raw = 0;
    reg mt6_fault = 0, mt6_sel = 1; reg [1:0] mt6_cause = 0; reg [15:0] mt6_max_no_good = 0;
    reg [15:0] mt68_frames = 0, mt68_good = 0, mt68_crc_err = 0, mt68_status_err = 0;
    reg [2:0] mt68_status = 0; reg [20:0] mt68_angle_raw = 0; reg mt68_fault = 0;
    wire hil_mode = 0; wire hil_tick; wire [15:0] hil_adc; wire signed [31:0] hil_enc;
    wire io_test_enable; wire signed [15:0] io_test_duty;
    wire signed [15:0] k1, k2, k3, k4, dz_pos, dz_neg, sw_amp, catch_ang, fall_ang,
                       relative_target, fling_delta, pos_k3, pos_k4, alpha_trim;

    debug_link dut (
        .clk(clk), .rst_n(rst_n), .tick(tick), .hil_mode(hil_mode),
        .hil_tick(hil_tick), .hil_adc(hil_adc), .hil_enc(hil_enc),
        .rx_valid(rx_valid), .rx_byte(rx_byte), .tx_start(tx_start),
        .tx_byte(tx_byte), .tx_busy(tx_busy),
        .mode(mode), .fault(fault), .otr_fault(otr_fault), .run_en(run_en),
        .calib_done(calib_done), .offset_c01(offset_c01), .adc(adc), .encoder(encoder),
        .diag_encoder(diag_encoder), .diag_adc(diag_adc),
        .diag_pos_trim(diag_pos_trim), .diag_alpha_pre(diag_alpha_pre),
        .diag_k3_eff(diag_k3_eff), .diag_k4_eff(diag_k4_eff),
        .diag_duty_pre_dz(diag_duty_pre_dz), .diag_obs_active(diag_obs_active),
        .diag_theta_e(diag_theta_e), .diag_term_k1(diag_term_k1), .diag_term_k2(diag_term_k2),
        .diag_term_k3(diag_term_k3), .diag_term_k4(diag_term_k4),
        .diag_pos_bias(diag_pos_bias), .diag_move_blend(diag_move_blend),
        .diag_settle_ms(diag_settle_ms), .diag_dz_idx(diag_dz_idx),
        .alpha(alpha), .alpha_dot(alpha_dot), .theta(theta), .theta_dot(theta_dot), .duty(duty),
        .reference(reference), .target(target),
        .local_stop(local_stop), .local_start(local_start),
        .k1(k1), .k2(k2), .k3(k3), .k4(k4), .dz_pos(dz_pos), .dz_neg(dz_neg),
        .sw_amp(sw_amp), .catch_ang(catch_ang), .fall_ang(fall_ang),
        .relative_target(relative_target), .fling_delta(fling_delta),
        .pos_k3(pos_k3), .pos_k4(pos_k4), .alpha_trim(alpha_trim),
        .remote_start(remote_start), .remote_stop(remote_stop), .remote_clear(remote_clear),
        .remote_mode(remote_mode), .remote_calib(remote_calib),
        .arm_stalled(arm_stalled),
        .io_test_enable(io_test_enable), .io_test_duty(io_test_duty),
        .move_pending(move_pending), .move_delta(move_delta), .config_lock(config_lock),
        .revision(revision), .rx_errors(rx_errors),
        .mt6_frames(mt6_frames), .mt6_good(mt6_good), .mt6_crc_err(mt6_crc_err),
        .mt6_mg_err(mt6_mg_err), .mt6_mg_code(mt6_mg_code), .mt6_angle_raw(mt6_angle_raw),
        .mt6_fault(mt6_fault), .mt6_sel(mt6_sel), .mt6_cause(mt6_cause),
        .mt6_max_no_good(mt6_max_no_good),
        .mt68_frames(mt68_frames), .mt68_good(mt68_good), .mt68_crc_err(mt68_crc_err),
        .mt68_status_err(mt68_status_err), .mt68_status(mt68_status),
        .mt68_angle_raw(mt68_angle_raw), .mt68_fault(mt68_fault)
    );

    always #5 clk = ~clk;
    // tick 高 20ns 覆盖 posedge
    always #1000 begin tick = 1; #20; tick = 0; end

    // 简易字节接收器：抓 tx_start 序列重组帧头
    integer got_bytes = 0;
    reg [7:0] hdr [0:7];
    reg [15:0] plen = 0;
    always @(posedge clk) begin
        if (tx_start) begin
            if (got_bytes < 8) begin
                hdr[got_bytes] = tx_byte;
                got_bytes = got_bytes + 1;
                if (got_bytes == 8) plen = {hdr[7], hdr[6]};
            end
        end
    end

    task send_byte(input [7:0] b);
        begin
            @(negedge clk); rx_valid = 1; rx_byte = b;
            @(negedge clk); rx_valid = 0;
        end
    endtask

    integer i;
    initial begin
        #100; rst_n = 1;
        repeat(5) @(posedge tick);
        // HELLO（seq=0，length=8，payload=SCHEMA+TOKEN）
        send_byte(8'h01); send_byte(8'h00);           // kind, seq lo
        send_byte(8'h00); send_byte(8'h08); send_byte(8'h00); // seq hi, len lo, len hi
        for (i = 0; i < 8; i = i + 1) send_byte(8'h00);       // payload（session 校验会拒，无妨）
        repeat(200) @(posedge clk);
        // 等 tick 触发 take_snapshot → TELEMETRY 帧
        repeat(3000) begin
            @(posedge clk);
            if (got_bytes >= 8) begin
                $display("帧头: kind=%02x seq=%02x%02x len=%0d",
                         hdr[0], hdr[7], hdr[6], {hdr[7], hdr[6]});
                if ({hdr[7], hdr[6]} == 88)
                    $display("PASS: 遥测帧 length=88");
                else
                    $display("FAIL: 遥测帧 length=%0d (期望 88)", {hdr[7], hdr[6]});
                $finish;
            end
        end
        $display("FAIL: 3000 拍内未抓到帧头");
        $finish;
    end
endmodule
