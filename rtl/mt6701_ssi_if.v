// ============================================================================
// mt6701_ssi_if.v — MT6701 SSI 绝对角接口（替换 ads1220_if + WDD35D4）
//
// 协议（数据手册 Rev1.8 §7.8 + SimpleFOC/官方 Arduino 例程三源交叉验证，
//   2026-10-15 定案）：
//   SPI mode 2 语义：CLK 空闲高。CSN↓ 后 ≥100ns 给首个 CLK↓——唤醒拍，
//   其后每个 CLK↑ 器件推出下一位（TDV≤15ns），CLK↓ 处采样。
//   完整帧 25 个下降沿 = 1 哑位 + D[13:0] + Mg[3:0] + CRC[5:0]，MSB 在前。
//   TCLK≥64ns、半高/低≥30ns、末CLK↓→CSN↑≥0.5×TCLK、帧间 CSN 高≥100ns。
//   （Rev1.8 把 Rev1.5 的帧前/帧后两个时序参数 TH/TL 命名对调过，涉及
//   手册字母时以功能描述为准。）官方 Arduino 库用 mode1+24 拍无哑位，
//   与本实现 mode2+25 拍是同一条总线波形，可互为旁证。
//
// 输出口径（2026-10-15 用户定案：全 360° 直读，弃 WDD35D4 的 345° 口径）：
//   adc_raw = angle14 << 2 ∈ 0..65532 ↔ 0..360°，1 LSB = 0.02197°；
//   搭配 ctrl_core 参数 ALPHA_SCALE_Q16=36000（t20k_top 传入，65536↔360.00°），
//   angle_proc 标定/回绕链路原样复用。
//
// 健康链（对齐 ads1220_if 的 adc_fault 语义，顶层接 otr_fault 安全链）：
//   坏帧 = CRC6 错 或 Mg 磁场健康告警（Mg[1:0]≠0 场强越界 / Mg[3]=1 超速
//   ——Rev1.8 语义 >55,000RPM，Rev1.5 旧称失跟踪，两义皆角度不可信；
//   Mg[2] 推按事件不算病）。连续 2 坏帧或 2ms 无好帧 → adc_fault=1，
//   期间 adc_raw 保持末好值；首个好帧自清。CRC 初值官方手册/例程均未注明
//   （两份官方驱动都跳过 CRC），按 0x00 实现——实机若角度正常却持续 fault，
//   先 CHECK_CRC=0 验证其余链路，再枚举 CRC_SEED 候选值。
// ============================================================================
`timescale 1ns / 1ps
module mt6701_ssi_if #(
    parameter integer SSI_HALF_DIV   = 4,      // 50MHz -> 半周期 80ns（6.25MHz）
    parameter integer TL_TICKS       = 2,      // CSN↓→首CLK↓ = 160ns ≥ 100ns
    parameter integer TH_TICKS       = 2,      // 末CLK↑→CSN↑ = 160ns ≥ 0.5×TCLK
    parameter integer FRAME_GAP_CLKS = (`CLK_HZ/10000), // 帧间隔 100µs（27MHz=2700）
    parameter integer POWERUP_CLKS   = (`CLK_HZ/500),   // 上电等待 2ms（27MHz=54000）
    parameter integer WATCHDOG_CLKS  = (`CLK_HZ/500),   // 好帧看门狗 2ms（27MHz=54000）
    parameter        CHECK_CRC      = 1'b1,    // CRC6 校验开关（排障用；勿用 bit 类型，Gowin 不支持）
    parameter        LATCH_ON_STRENGTH = 1'b1, // Mg[1:0] 场强告警是否计入安全链坏帧累计
                                               //  （默认旧行为；T20K=0，理由见 latch_bad）
    parameter        LATCH_ON_TRACK    = 1'b1, // Mg[3] 失跟踪是否计入安全链坏帧累计
                                               //  （默认旧行为；T20K=0——23:29 cause=3
                                               //   定案：换向期线圈场扰短暂丢锁成串
                                               //   （Mg +207、CRC 恒 0、角度事后有效），
                                               //   持续丢锁由 2ms 好帧看门狗兜底）
    // Mg 告警帧的角度是否照常采纳（2026-10-23 用户定案）。
    // 现状（0）：frame_good 要求 场强+失跟踪 全 OK，否则保持旧角 ⇒ 满舵期间
    //   MT6701 反复"冻结"7~10ms（实测），θ̇ 塌 0，刹车只能靠时间判、不能闭环。
    // 打开（1）：数据门只看 CRC（完整性硬校验），Mg 告警只保留计数/诊断。
    //   实板证据：冻结期间原始角也不动（传感器在保持），解冻跳变 2/3 与累积
    //   转角一致（<0.6°）、1/3 最多 3.5° ⇒ 告警帧的角度基本可用。
    parameter        ACCEPT_MG_WARN = 1'b0,
    parameter integer MG_WARN_MAX   = 200,     // 连续 Mg 告警帧上限（≈20ms @9.6kHz）
                                               //  超过则仍判故障（防磁铁真丢失）
    parameter [5:0]  CRC_SEED       = 6'd0,    // CRC6 初值（手册未注明，官方例程亦跳过 CRC；
                                               //  全帧判错时与 CHECK_CRC=0 配合枚举候选）
    parameter [3:0]  BAD_FRAMES_LATCH = 4'd2    // 连续坏帧锁存门限（2026-10-19 加固）：
                                               //  T20K 实测三连锁存均在稳摆 duty 近零/
                                               //  换向密集区间（换向尖峰短促 EMI 打掉
                                               //  1~几帧），稳定大 duty 驱动 10s 零坏帧。
                                               //  默认 2 保持旧行为；T20K 放宽 8（约
                                               //  200µs）容忍突发，真断线/持续损坏仍由
                                               //  2ms 好帧看门狗兜底。计数器不受影响。
)(
    input  wire        clk,
    input  wire        rst_n,
    output reg         ssi_csn,
    output reg         ssi_clk,
    input  wire        ssi_do,
    output reg  [15:0] adc_raw,
    output reg  [13:0] angle14_o,   // 原生 14 位绝对角（experiment/mt6701-theta 电机
                                    // 侧 θ 链消费；与 adc_raw 同拍发布/同 hold 语义）
    output reg         adc_valid,
    output reg         adc_fault,
    // ---- 链路健康诊断（STATUS 应答导出，2026-10-19）----
    // 全部为粘滞/快照量，供上位机「传感诊断」页读：磁场强度码 Mg[1:0] 与
    // 失跟踪 Mg[3] 直接反映径向磁铁叠装的干扰程度，是 bring-up 的关键判据。
    output reg  [15:0] st_frames,      // 帧计数（取低 16 位，饱和回绕）
    output reg  [15:0] st_good,        // 好帧计数（低 16 位）
    output reg  [15:0] st_crc_err,     // CRC6 错帧计数（饱和到 0xFFFF）
    output reg  [15:0] st_mg_err,      // Mg 健康告警帧计数（饱和到 0xFFFF）
    output reg  [2:0]  st_mg_code,     // 最近帧 Mg[1:0] + Mg[3]：{lost_track, mg1, mg0}
    output reg  [13:0] st_angle_raw,   // 最近帧原始角（未做 hold，便于看毛刺）
    output reg  [1:0]  st_fault_cause, // 最近一次锁存原因：0 无 1 CRC 2 场强 3 失跟踪
                                    // 0 = 看门狗路径（≥WATCHDOG_CLKS 无好帧）
    output reg  [15:0] st_max_no_good, // 最长连续无好帧段（帧数，×帧周期≈时长；
                                    // 2026-10-20 12:40 cause=0 揭示看门狗路径在
                                    // 发火=存在 ms 级连续告警段，此计数直接量出
                                    // 扰动段真实长度，标定看门狗放宽是否足够）
    output wire        st_frame_pulse  // 每好帧一个 clk 脉冲（上位机/示波观察用）
);
    localparam [2:0] ST_POWERUP = 3'd0;
    localparam [2:0] ST_TL      = 3'd1;
    localparam [2:0] ST_BIT     = 3'd2;
    localparam [2:0] ST_TH      = 3'd3;
    localparam [2:0] ST_GAP     = 3'd4;

    // 位内相位：SAMP=CLK↓ 后 1 拍采样；RISE/DROP=半周期点翻转 CLK
    localparam [1:0] PH_SAMP = 2'd0;
    localparam [1:0] PH_RISE = 2'd1;
    localparam [1:0] PH_DROP = 2'd2;

    reg [2:0]  state;
    reg [1:0]  phase;
    reg [7:0]  spi_cnt;
    reg [4:0]  bit_idx;       // 采样序号：0=哑位，1..24=数据
    reg [24:0] sample_sr;     // 25 采样，[24]=哑位，[23:0]=帧
    reg [7:0]  tl_cnt, th_cnt;
    reg [19:0] gap_cnt, pw_cnt, wd_cnt;
    reg [3:0]  bad_streak;
    reg [15:0] no_good_run;   // 当前连续无好帧段长度（帧数）
    reg [15:0] mg_warn_run;   // 当前连续 Mg 告警帧数（ACCEPT_MG_WARN 的兜底用）
    reg        good_pulse;
    assign     st_frame_pulse = good_pulse;

    wire spi_tick = (spi_cnt == SSI_HALF_DIV - 1);

    wire [23:0] frame_rx   = sample_sr[23:0];
    wire [13:0] angle_rx   = frame_rx[23:10];
    wire [5:0]  crc_calc   = crc6(frame_rx);
    wire        crc_ok     = (crc_calc == frame_rx[5:0]);
    wire        mg_strength_ok = (frame_rx[7:6] == 2'b00);   // Mg[1:0] 场强
    wire        mg_track_ok    = !frame_rx[9];               // Mg[3] 失跟踪
    // 数据门（角度是否采纳）：CRC+场强+失跟踪恒全查——所有实机验证（斜率/方向）
    // 都建立在这条保守路径上；Mg 告警帧一律保持旧角（宁可 ~200µs 陈旧，不采纳
    // 存疑角度），frame_good 只在好帧时更新角度/清看门狗。
    // 数据门（角度是否采纳）：默认 CRC+场强+失跟踪全查。
    // ACCEPT_MG_WARN=1 时只查 CRC —— Mg 告警帧的角度照常采纳（不再保持旧角），
    // 冻结消失、θ̇ 连续，才有条件把刹车/捕获改成闭环判据。
    wire        mg_data_ok  = ACCEPT_MG_WARN ? 1'b1 : (mg_strength_ok && mg_track_ok);
    wire        frame_good = (CHECK_CRC ? crc_ok : 1'b1) && mg_data_ok;
    // 安全链门（坏帧是否累计向锁存）：与数据门分离（2026-10-19 23:29 定案）。
    // Mg 在电机旁是"意见"——换向线圈场扰的瞬时越限/丢锁，事后角度被证明有效；
    // T20K 双关（场强+失跟踪都无锁存权），只留 CRC。真失效（断线/磁铁脱落/
    // 持续干扰）= 连续 2ms 无 frame_good → 看门狗照旧锁 SAFE。
    wire        latch_bad = (CHECK_CRC ? !crc_ok : 1'b0)
                        || (LATCH_ON_STRENGTH ? !mg_strength_ok : 1'b0)
                        || (LATCH_ON_TRACK    ? !mg_track_ok    : 1'b0);

    // CRC-6：x^6 + x + 1，MSB 先行，初值 CRC_SEED，覆盖 {D[13:0], Mg[3:0]}
    function [5:0] crc6(input [23:0] f);
        reg [5:0] c;
        integer i;
        begin
            c = CRC_SEED;
            for (i = 23; i >= 6; i = i - 1) begin
                if (c[5] ^ f[i]) c = {c[4:0], 1'b0} ^ 6'h03;
                else             c = {c[4:0], 1'b0};
            end
            crc6 = c;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_POWERUP;
            phase      <= PH_SAMP;
            spi_cnt    <= 8'd0;
            bit_idx    <= 5'd0;
            sample_sr  <= 25'd0;
            tl_cnt     <= 8'd0;
            th_cnt     <= 8'd0;
            gap_cnt    <= 20'd0;
            pw_cnt     <= 20'd0;
            wd_cnt     <= 20'd0;
            bad_streak <= 4'd0;
            no_good_run <= 16'd0;
            st_max_no_good <= 16'd0;
            ssi_csn    <= 1'b1;
            ssi_clk    <= 1'b1;   // SPI mode 2：空闲高
            adc_raw    <= 16'd0;
            angle14_o  <= 14'd0;
            adc_valid  <= 1'b0;
            adc_fault  <= 1'b0;
            st_frames  <= 16'd0;
            st_good    <= 16'd0;
            st_crc_err <= 16'd0;
            st_mg_err  <= 16'd0;
            st_mg_code <= 3'b000;
            st_angle_raw <= 14'd0;
            st_fault_cause <= 2'd0;
            good_pulse <= 1'b0;
        end else begin
            adc_valid <= 1'b0;
            good_pulse <= 1'b0;

            if (spi_tick) spi_cnt <= 8'd0;
            else          spi_cnt <= spi_cnt + 8'd1;

            case (state)
                ST_POWERUP: begin
                    ssi_csn <= 1'b1;
                    ssi_clk <= 1'b1;
                    if (pw_cnt >= POWERUP_CLKS - 1) begin
                        pw_cnt <= 20'd0;
                        tl_cnt <= 8'd0;
                        wd_cnt <= 20'd0;
                        state  <= ST_TL;
                    end else begin
                        pw_cnt <= pw_cnt + 20'd1;
                    end
                end

                // TL：CSN 已低，等够 CSN↓→首CLK↓ 再给唤醒拍
                ST_TL: begin
                    ssi_csn <= 1'b0;
                    ssi_clk <= 1'b1;
                    wd_cnt  <= wd_cnt + 20'd1;
                    if (spi_tick) begin
                        if (tl_cnt >= TL_TICKS - 1) begin
                            ssi_clk <= 1'b0;   // 首 CLK↓ = 哑位拍
                            tl_cnt  <= 8'd0;
                            bit_idx <= 5'd0;
                            phase   <= PH_SAMP;
                            state   <= ST_BIT;
                        end else begin
                            tl_cnt <= tl_cnt + 8'd1;
                        end
                    end
                end

                ST_BIT: begin
                    ssi_csn <= 1'b0;
                    wd_cnt  <= wd_cnt + 20'd1;
                    case (phase)
                        PH_SAMP: begin
                            // CLK↓ 后 1 拍：数据自上一 CLK↑ 起已稳定 ≥ 半周期
                            sample_sr <= {sample_sr[23:0], ssi_do};
                            phase     <= PH_RISE;
                        end
                        PH_RISE: begin
                            if (spi_tick) begin
                                ssi_clk <= 1'b1;
                                if (bit_idx == 5'd24) begin
                                    // 第 25 个 CLK↑ = 帧末沿，发布评估结果
                                    st_frames     <= st_frames + 16'd1;
                                    st_mg_code    <= {frame_rx[9], frame_rx[7:6]};
                                    st_angle_raw  <= angle_rx;
                                    if (!crc_ok && st_crc_err != 16'hFFFF)
                                        st_crc_err <= st_crc_err + 16'd1;
                                    if ((!mg_strength_ok || !mg_track_ok) && st_mg_err != 16'hFFFF)
                                        st_mg_err  <= st_mg_err + 16'd1;
                                    if (frame_good) begin
                                        adc_raw   <= {angle_rx, 2'b00};
                                        angle14_o <= angle_rx;
                                        adc_valid <= 1'b1;
                                        adc_fault <= 1'b0;
                                        bad_streak<= 4'd0;
                                        no_good_run <= 16'd0;
                                        wd_cnt    <= 20'd0;
                                        st_good   <= st_good + 16'd1;
                                        good_pulse<= 1'b1;
                                    end else begin
                                        // 无好帧段长度测量（不分锁存权，纯链路质量）
                                        no_good_run <= no_good_run + 16'd1;
                                        if (no_good_run + 16'd1 > st_max_no_good)
                                            st_max_no_good <= no_good_run + 16'd1;
                                    end
                                    if (latch_bad) begin
                                        bad_streak <= bad_streak + 4'd1;
                                        if (bad_streak + 4'd1 >= BAD_FRAMES_LATCH) begin
                                            // 锁存原因快照（2026-10-19 22:54 分叉：
                                            // 场强/失跟踪在 st_mg_err 里混数，无法
                                            // 区分降级是否生效）：首个越限帧定罪，
                                            // 1=CRC 2=场强 3=失跟踪，仅上升沿写入
                                            if (!adc_fault)
                                                st_fault_cause <= crc_ok ?
                                                    (mg_strength_ok ? 2'd3 : 2'd2) : 2'd1;
                                            adc_fault <= 1'b1;
                                        end
                                    end else begin
                                        bad_streak <= 4'd0;  // Mg 告警但无锁存权：仅保持角度
                                    end
                                    // Mg 持续告警兜底（仅 ACCEPT_MG_WARN=1 时生效）：
                                    // 短暂告警（换向 EMI，实测 7~10ms）照常采纳角度；
                                    // 连续 ≥ MG_WARN_MAX 帧（默认 200 帧 ≈ 20ms @9.6kHz）
                                    // 说明磁铁真的丢了/持续干扰 ⇒ 仍然锁故障。
                                    if (!mg_strength_ok || !mg_track_ok) begin
                                        if (mg_warn_run < 16'd65535)
                                            mg_warn_run <= mg_warn_run + 16'd1;
                                        if (ACCEPT_MG_WARN && (mg_warn_run >= MG_WARN_MAX))
                                            adc_fault <= 1'b1;
                                    end else
                                        mg_warn_run <= 16'd0;
                                    th_cnt <= 8'd0;
                                    state  <= ST_TH;
                                end else begin
                                    bit_idx <= bit_idx + 5'd1;
                                    phase   <= PH_DROP;
                                end
                            end
                        end
                        PH_DROP: begin
                            if (spi_tick) begin
                                ssi_clk <= 1'b0;
                                phase   <= PH_SAMP;
                            end
                        end
                        default: phase <= PH_SAMP;
                    endcase
                end

                ST_TH: begin
                    ssi_clk <= 1'b1;
                    wd_cnt  <= wd_cnt + 20'd1;
                    if (spi_tick) begin
                        if (th_cnt >= TH_TICKS - 1) begin
                            ssi_csn <= 1'b1;
                            th_cnt  <= 8'd0;
                            gap_cnt <= 20'd0;
                            state   <= ST_GAP;
                        end else begin
                            th_cnt <= th_cnt + 8'd1;
                        end
                    end
                end

                ST_GAP: begin
                    ssi_csn <= 1'b1;
                    ssi_clk <= 1'b1;
                    wd_cnt  <= wd_cnt + 20'd1;
                    if (gap_cnt >= FRAME_GAP_CLKS - 1) begin
                        gap_cnt <= 20'd0;
                        tl_cnt  <= 8'd0;
                        state   <= ST_TL;
                    end else begin
                        gap_cnt <= gap_cnt + 20'd1;
                    end
                end

                default: state <= ST_POWERUP;
            endcase

            // 自愈：好帧看门狗超时 = 链路死（断线/卡帧），强制放弃当前帧回 GAP
            if (state != ST_POWERUP && wd_cnt >= WATCHDOG_CLKS - 1) begin
                adc_fault <= 1'b1;
                state     <= ST_GAP;
                phase     <= PH_SAMP;
                bit_idx   <= 5'd0;
                ssi_csn   <= 1'b1;
                ssi_clk   <= 1'b1;
                gap_cnt   <= 20'd0;
                wd_cnt    <= 20'd0;
            end
        end
    end
endmodule
