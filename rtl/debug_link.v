// V2 first release: immutable telemetry + acknowledged safe-state transactions.
// One outstanding request per session; identical immediate retries replay cached
// response. Older/wrong sequence numbers never repeat a relative move.
//
// HIL lockstep (hil_mode=1，DE10 无硬件算法验证用): STEP 命令携带上位机被控
// 对象算出的传感器字 (adc u16, enc i32)；收到后锁存并产生一个 tick 脉冲，
// 恰好推进一个仿真毫秒。遥测改为每拍一帧，采样点后移一拍，以便包含本拍算出的
// duty。上位机凭 ACK 确认推进、从遥测取回 duty 再积分下一步，故轨迹与 1kHz
// 仿真实跑逐位一致，只是墙钟变慢。hil_mode=0 时 STEP 返回 STATE，遥测内容与
// 采样时序与旧版逐比特一致。
//
// HELLO 回包携带三个 u32（见 HELLO 分支）：schema / build / capabilities。
//   schema        参数表指纹 DEBUG_SCHEMA，不匹配即拒握手（BAD_SCHEMA）。
//   build         {板标识, 构建日期 YYMMDD}——板标识由
//                 tools/generate_debug_config.py 写入 DEBUG_BOARD_ID，
//                 上位机据此显示板名，无需另设"我在哪块板"的配置。
//   capabilities  能力位，由 hil_mode 二选一（DEBUG_CAPS_REAL / DEBUG_CAPS_HIL）。
//                 上位机按能力位决定显示哪些后端与按钮；RTL 与 host 里都不允许
//                 出现按板名分支——板级差异只能通过 build + 能力位表达。
//
//   位   名称            含义
//   bit0 CAP_PARAMS      参数读/写/事务
//   bit1 CAP_MOTION      动作与模式命令（STOP/CLEAR/START/MOVE/BALANCE）
//   bit2 CAP_TELEMETRY   周期遥测
//   bit3 CAP_CALIB_DC    静止下垂零点标定
//   bit4 CAP_STEP_LOCK   STEP 锁步（传感器由上位机提供、逐拍推进）
//   bit5 CAP_HW_SENSORS  硬件角度/编码器有效
//   bit6 CAP_MOTOR_OUT   真实电机输出有效
//   bit7 CAP_IO_TEST     独立 I/O 联调命令有效
`include "j280_debug_params.vh"
module debug_link (
    input wire clk,rst_n,tick,
    input wire hil_mode,
    output reg hil_tick,output reg [15:0] hil_adc,output reg signed [31:0] hil_enc,
    input wire rx_valid,input wire [7:0] rx_byte,
    output wire tx_start,output wire [7:0] tx_byte,input wire tx_busy,
    input wire [2:0] mode,input wire fault,otr_fault,run_en,calib_done,
    input wire signed [15:0] alpha,alpha_dot,theta_dot,duty,offset_c01,
    input wire signed [31:0] theta,reference,target,encoder,
    input wire signed [31:0] diag_encoder,
    input wire [15:0] adc,diag_adc,
    // ---- 遥测诊断量（记录格式 v3，2026-10-21）----
    input  wire signed [15:0] diag_pos_trim,     // POS 积分器输出 duty
    input  wire signed [15:0] diag_alpha_pre,    // trim 前的 α（当轮真实零偏）
    input  wire signed [15:0] diag_k3_eff,       // 渐变后实际 K3
    input  wire signed [15:0] diag_k4_eff,       // 渐变后实际 K4
    input  wire signed [15:0] diag_duty_pre_dz,  // 死区补偿前的指令
    input  wire               diag_obs_active,   // 观测器是否接管
    // v4 遥测（2026-10-27）：θe + 四个分项（term 恒 0，上位机重建）
    input  wire signed [15:0] diag_theta_e,
    input  wire signed [15:0] diag_term_k1,
    input  wire signed [15:0] diag_term_k2,
    input  wire signed [15:0] diag_term_k3,
    input  wire signed [15:0] diag_term_k4,
    // v5 遥测（2026-10-31）：原始量测链（观测器 mux 之前）+ 精确增益 + 积分器分量
    input wire signed [31:0] diag_theta_raw,
    input wire signed [15:0] diag_alpha_raw,
    input wire signed [15:0] diag_theta_dot_raw,
    input wire signed [15:0] diag_alpha_dot_raw,
    input wire signed [15:0] diag_k1_eff,
    input wire signed [15:0] diag_k2_eff,
    input wire signed [15:0] diag_pos_i_trim,
    // v6 遥测（2026-11-02）：级联/降额/稳定窗/LUT 内部状态
    input wire signed [15:0] diag_pos_bias,
    input wire [15:0] diag_move_blend,
    input wire [15:0] diag_settle_ms,
    input wire [3:0] diag_dz_idx,
    input wire local_stop,local_start,
    input wire arm_stalled,   // 臂真停（arm_vel 判停；MOVE 入场确定性门，2026-09-30）
    output wire signed [15:0] k1,k2,k3,k4,dz_pos,dz_neg,sw_amp,catch_ang,fall_ang,relative_target,fling_delta,
    output wire signed [15:0] pos_k3,pos_k4,
    output wire signed [15:0] alpha_trim,   // id13 α 零位微调（运行时，替代自动自校准）
    output reg remote_start,remote_stop,remote_clear,remote_mode,remote_calib,
    output reg remote_force_pos,   // 强制 BAL→POS（绕过 pos_ready，保持 200 拍）
    output reg vel_demo_en,        // 速度曲线演示使能（固件内生梯形 v_ref）
    output reg signed [15:0] vel_demo_v,   // 目标速度（0.1°/s）
    output reg signed [15:0] vel_demo_a,   // 斜率（0.1°/s per ms）
    output reg move_pending,output reg signed [31:0] move_delta,
    output reg move_is_abs,     // 1=move_delta 是绝对目标（不是相对位移）
    output reg io_test_enable,output reg signed [15:0] io_test_duty,
    output wire config_lock,
    output reg [31:0] revision,
    output wire [31:0] rx_errors,
    // ---- MT6701 链路健康（STATUS 应答尾追 12 字节；无该传感的板卡恒 0）----
    // 载荷布局（自 STATUS 原 31 字节之后）：
    //   +0  u16 frames        +2  u16 good
    //   +4  u16 crc_err       +6  u16 mg_err
    //   +8  u8  mg_code       +9  u8 flags{bit0 fault,bit1 selected}
    //   +10 u16 angle_raw(14b)
    input wire [15:0] mt6_frames, mt6_good, mt6_crc_err, mt6_mg_err,
    input wire [2:0] mt6_mg_code, input wire [13:0] mt6_angle_raw,
    input wire mt6_fault, mt6_sel,
    input wire [1:0] mt6_cause,     // MT6701 最近锁存原因（塞 flags 字节 bit7:6）
    input wire [15:0] mt6_max_no_good, // MT6701 最长连续无好帧段（帧数，STATUS 再+2B）
    // ---- MT6835（摆侧 α）链路健康（STATUS 应答再尾追 14 字节；总 57）----
    // 载荷布局（自 STATUS 43 字节之后）：
    //   +0  u16 frames         +2  u16 good
    //   +4  u16 crc_err        +6  u16 status_err
    //   +8  u8  status(3b)     +9  u8 flags{bit0 fault}
    //   +10 u32 angle_raw(21b)
    input wire [15:0] mt68_frames, mt68_good, mt68_crc_err, mt68_status_err,
    input wire [2:0] mt68_status, input wire [20:0] mt68_angle_raw,
    input wire mt68_fault
);
    localparam HELLO=1,READ=2,BEGIN_TXN=3,SET=4,COMMIT=5,ABORT=6,DEFAULTS=7,
        STOP=8,CLEAR=9,START=10,MOVE=11,BALANCE=12,CALIBRATE=13,STATUS=14,STEP=15,
        IO_TEST=16,FORCE_POS=17,VEL_DEMO=18,MOVE_ABS=20;
    localparam OK=0,BAD_LENGTH=1,BAD_ID=2,RANGE=3,STATE=4,CONFLICT=5,
        NO_TXN=6,BAD_SCHEMA=7,EXPIRED=8,SEQUENCE=9;
    reg signed [15:0] active[0:13],shadow[0:13];
    // ★参数寄存器复位（2026-11-04 修复）：此前 active/shadow 无复位逻辑，
    // 仿真中初值为 X 并一路传播到 pos_tgt / duty（tb_top_smoke 实测 duty=X）。
    // 实板靠 SRAM 上电随机值 + 上位机首次写参掩盖，但仿真与冷启动都不可靠。
    // 复位值 = ROM 默认值（j280_debug_params.vh 的 DP_n_DEFAULT），
    // 与 DEFAULTS 命令回退的目标一致。
    integer pi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active[0]  <= `DP_0_DEFAULT;  active[1]  <= `DP_1_DEFAULT;
            active[2]  <= `DP_2_DEFAULT;  active[3]  <= `DP_3_DEFAULT;
            active[4]  <= `DP_4_DEFAULT;  active[5]  <= `DP_5_DEFAULT;
            active[6]  <= `DP_6_DEFAULT;  active[7]  <= `DP_7_DEFAULT;
            active[8]  <= `DP_8_DEFAULT;  active[9]  <= `DP_9_DEFAULT;
            active[10] <= `DP_10_DEFAULT; active[11] <= `DP_11_DEFAULT;
            active[12] <= `DP_12_DEFAULT; active[13] <= `DP_13_DEFAULT;
            for (pi = 0; pi < 14; pi = pi + 1) shadow[pi] <= 16'sd0;
        end
    end
    assign k1=active[0];assign k2=active[1];assign k3=active[2];assign k4=active[3];
    // Existing out_stage consumes twice the actual deadband compensation.
    assign dz_pos=active[4]<<<1;assign dz_neg=active[5]<<<1;assign sw_amp=active[6];
    assign catch_ang=active[7];assign fall_ang=active[8];assign relative_target=active[9];
    assign fling_delta=active[10];
    assign pos_k3=active[11];assign pos_k4=active[12];
    // id13 α 零位微调（2026-10-20）：叠加进 offset_c01 的运行时偏置，替代
    // 自动自校准（AZ）。标定后由上位机写入，不烧板即可调零位；CAL 仍可用
    // （CAL 重写 offset_c01，trim 保留，两者相加）。
    assign alpha_trim=active[13];
    reg transaction,commit_pending,txn_expired;
    reg [12:0] txn_age;
    reg [31:0] session,tick_count;
    reg [15:0] last_seq;
    reg [7:0] last_kind;
    reg [15:0] last_length;
    reg [511:0] last_request,cached_reply,reply;
    reg [15:0] reply_seq,cached_length,reply_length;
    reg reply_ready,have_last;
    reg [15:0] pending_seq;
    reg [7:0] pending_kind;
    reg [511:0] pending_request;
    reg [15:0] pending_length;
    reg [15:0] telemetry_seq;
    reg period;
    reg tick_d;                  // tick 延迟一拍（HIL 遥测采样点对齐 duty 更新）
    reg step_pending;            // STEP 已受理，等控制链稳定后应答
    reg [3:0] step_wait;
    reg [15:0] step_seq,step_len;
    reg [511:0] step_req;
    // ---- MOVE 排队执行器（2026-11-01 P1，移植 champion 2026-10-27 机构）----
    // P0 板测：稳摆时臂持续微调（~6°/s 永不停），MOVE 的"臂停或<15°/s"门
    // 首发受理率 0/3——命令层直接拒绝=上位机只能碰运气。改为：硬拒绝只留
    // 故障/配置锁/急停，其余一律受理并锁存目标，门一开（move_exec_q）就发脉冲。
    // 门与 mode_fsm pos_ready 同口径（09-30 教训：两门不一致→"受理了却不切换"）。
    // 门控寄存一拍（比较树直接进命令分发会拖 Fmax；传感量 1ms 级慢变无影响）。
    wire move_exec = (mode==2 || mode==3) && !fault && !config_lock && !local_stop &&
                     (alpha > -16'sd500) && (alpha < 16'sd500) &&
                     (alpha_dot > -16'sd300) && (alpha_dot < 16'sd300) &&
                     (mode==3 || ((theta_dot > -16'sd600) && (theta_dot < 16'sd600) &&
                      (arm_stalled || (theta_dot > -16'sd400 && theta_dot < 16'sd400))));
    reg move_exec_q, move_queued;
    reg move_q_is_abs;          // 1=MOVE_ABS（move_q_delta 存绝对目标）
    // move_is_abs 已在端口列表声明为 output reg，此处不得重复声明
    reg signed [31:0] move_q_delta;
    reg [7:0] move_hold;         // move_pending 保持拍数（POS 内让 ctrl_core 采到）
    reg [7:0] remote_mode_hold;  // remote_mode 保持拍数（2026-11-03 NO_POS 修复）
    reg [7:0] force_hold;        // remote_force_pos 保持拍数（强制进 POS 用）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) move_exec_q <= 1'b0;
        else        move_exec_q <= move_exec;
    end
    // 88 字节（记录格式 v6 = v5 80 + 8：pos_bias s16/move_blend u16/settle_ms u16/
    // {dropped u16 高 4 位=pad:4 低=dz_idx:4}，2026-11-02）。2026-10-27 教训：位宽必须跟着快照拼接一起扩，
    // 否则高位被静默截断（拼接改了、寄存器没改）。
    reg [703:0] snapshot;
    reg snap_ready;
    // 遥测载荷字节数。旧布局正好 40 字节（320 位，无填充）；v3 追加
    //   5×s16 (pos_trim/alpha_pre/k3_eff/k4_eff/duty_pre_dz) = 10
    //   + u8 obs_active + s16 alpha_trim + u8 对齐             = 4
    // ⇒ 40 + 13 + 1 = 54 字节；v4 +10 = 64；v5 +16 = 80。
    // 上位机侧 V5 解包与 SNAPSHOT_SIZES 白名单已就绪（lab/上位机/host）。
    localparam [15:0] SNAPSHOT_BYTES = 16'd88;
    reg [31:0] dropped;
    wire safe=(mode==0 || mode==4);
    assign config_lock=transaction || commit_pending;
    wire decoded_valid;
    reg frame_valid;
    wire [7:0] frame_kind;
    wire [15:0] frame_seq,frame_length;
    wire [511:0] frame_payload;
    packet_rx rx(clk,rst_n,rx_valid,rx_byte,decoded_valid,frame_kind,frame_seq,frame_length,frame_payload,rx_errors);
    // 512-bit equality otherwise maps to a long carry/reduction chain. Compare
    // 32-bit blocks in parallel, then register the reduction before dispatch.
    reg [15:0] equal_blocks;
    reg duplicate_payload;
    reg [1:0] decode_delay;
    integer block_index;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin equal_blocks<=0;duplicate_payload<=0;decode_delay<=0;frame_valid<=0;end
        else begin
            decode_delay<={decode_delay[0],decoded_valid};
            frame_valid<=decode_delay[1];
            if(decoded_valid)
                for(block_index=0;block_index<16;block_index=block_index+1)
                    equal_blocks[block_index]<=frame_payload[block_index*32+:32]==last_request[block_index*32+:32];
            duplicate_payload<=&equal_blocks;
        end
    end
    reg send;
    reg [7:0] send_kind;
    reg [15:0] send_seq,send_length;
    reg [703:0] send_payload;   // v6：与 snapshot 同宽 704 位
    wire sender_busy;
    packet_tx tx(clk,rst_n,send,send_kind,send_seq,send_length,send_payload,
                 sender_busy,tx_start,tx_byte,tx_busy);
    wire [15:0] param_id=frame_payload[47:32];
    wire signed [31:0] param_value=$signed(frame_payload[79:48]);
    wire signed [31:0] requested_delta=$signed(frame_payload[63:32]);
    // MOVE_ABS：绝对目标（c01）→ 用【当下 θ】算最短路 delta = wrap180(target−θ)。
    // 治"上位机数据陈旧导致终点偏"（2026-11-04 用户实测点 30° 只到 20°）。
    // 上位机先把目标取成"离当前 θ 最近的 360° 等价角"（它知道大致 θ），
    // 固件只做一次 ±360° 修正——避免 34 位取模（实测 Fmax 掉到 15.7MHz）。
    wire signed [33:0] abs_span   = $signed({{2{requested_delta[31]}}, requested_delta})
                                  - $signed({{2{theta[31]}}, theta});
    wire signed [33:0] abs_delta  = (abs_span >  33'sd18000) ? abs_span - 34'sd36000 :
                                    (abs_span < -33'sd18000) ? abs_span + 34'sd36000 : abs_span;
    wire signed [32:0] requested_end=$signed({theta[31],theta})+
                                     $signed({requested_delta[31],requested_delta});
    integer i;
    reg [7:0] code;
    reg [511:0] answer;
    reg [15:0] answer_length;
    function signed [31:0] default_value(input integer p);
        begin case(p)
            0:default_value=`DP_0_DEFAULT;1:default_value=`DP_1_DEFAULT;
            2:default_value=`DP_2_DEFAULT;3:default_value=`DP_3_DEFAULT;
            4:default_value=`DP_4_DEFAULT;5:default_value=`DP_5_DEFAULT;
            6:default_value=`DP_6_DEFAULT;7:default_value=`DP_7_DEFAULT;
            8:default_value=`DP_8_DEFAULT;9:default_value=`DP_9_DEFAULT;
            10:default_value=`DP_10_DEFAULT;
            11:default_value=`DP_11_DEFAULT;12:default_value=`DP_12_DEFAULT;
            13:default_value=`DP_13_DEFAULT;
            default:default_value=0;
        endcase end
    endfunction
    function valid_value(input [15:0] p,input signed [31:0] v);
        begin case(p)
            0:valid_value=v>=`DP_0_MIN && v<=`DP_0_MAX;
            1:valid_value=v>=`DP_1_MIN && v<=`DP_1_MAX;
            2:valid_value=v>=`DP_2_MIN && v<=`DP_2_MAX;
            3:valid_value=v>=`DP_3_MIN && v<=`DP_3_MAX;
            4:valid_value=v>=`DP_4_MIN && v<=`DP_4_MAX;
            5:valid_value=v>=`DP_5_MIN && v<=`DP_5_MAX;
            6:valid_value=v>=`DP_6_MIN && v<=`DP_6_MAX;
            7:valid_value=v>=`DP_7_MIN && v<=`DP_7_MAX;
            8:valid_value=v>=`DP_8_MIN && v<=`DP_8_MAX;
            9:valid_value=v>=`DP_9_MIN && v<=`DP_9_MAX;
            10:valid_value=v>=`DP_10_MIN && v<=`DP_10_MAX;
            11:valid_value=v>=`DP_11_MIN && v<=`DP_11_MAX;
            12:valid_value=v>=`DP_12_MIN && v<=`DP_12_MAX;
            13:valid_value=v>=`DP_13_MIN && v<=`DP_13_MAX;
            default:valid_value=0;
        endcase end
    endfunction
    // Response prefix: command u8, status u8, revision u32, applied_tick u32, session u32.
    task complete_reply(input [7:0] command,result,input [15:0] sequence_number,
                        input [15:0] request_length,input [511:0] request_payload,
                        input [511:0] body,input [15:0] size,input [31:0] rev,stamp,token);
        reg [511:0] r;
        begin
            r=body;r[7:0]=command;r[15:8]=result;r[47:16]=rev;r[79:48]=stamp;r[111:80]=token;
            reply<=r;reply_seq<=sequence_number;reply_length<=size;reply_ready<=1;
            cached_reply<=r;cached_length<=size;last_seq<=sequence_number;
            last_kind<=command;last_length<=request_length;last_request<=request_payload;have_last<=1;
        end
    endtask
    // Snapshot layout: HIL 路径复用同一帧格式，上位机两种模式只解一种帧。
    task take_snapshot;
        begin
            if(snap_ready)dropped<=dropped+1;
            // 布局（LSB→MSB，小端逐个 u8 发出）：
            //   +0 u32 tick  +4 u32 revision  +8 u8 mode  +9 u8 fault
            //   +10 s16 alpha  +12 s16 alpha_dot  +14 i32 theta  +18 s16 theta_dot
            //   +20 s16 duty  +22 i32 reference  +26 i32 target  +30 i32 encoder
            //   +34 u16 adc  +36 s16 offset  +38 u8 calib_done  +39 u8 run_en
            // v3 追加（+40..+53）：
            //   +40 s16 pos_trim  +42 s16 alpha_pre  +44 s16 k3_eff
            //   +46 s16 k4_eff  +48 s16 duty_pre_dz  +50 u8 obs_active
            //   +51 s16 alpha_trim  +53 u8 对齐
            // v4 追加（+54..+63）：term_k1..k4（RTL 恒 0，上位机重建）+ θe
            // v5 追加（+64..+79，2026-10-30 布局，本树 11-01 同步）：原始量测链
            //   （观测器 mux 之前）+ 精确增益 + 积分器分量：
            //   +64 s32 theta_raw   +68 s16 alpha_raw    +70 s16 theta_dot_raw
            //   +72 s16 alpha_dot_raw  +74 s16 k1_eff   +76 s16 k2_eff
            //   +78 s16 pos_i_trim（积分器分量；本树无 AT 分离，恒=pos_trim）
            // v6 段 8B：pos_bias s16 | move_blend u16 | settle_ms u16 | {dz_idx:4,dropped_lo:4}
            // dropped 只取低 4 位（饱和计数器，>15 已无诊断意义）——Bx 填充留字节对齐。
            snapshot<={6'd0,dropped[3:0],diag_dz_idx,diag_settle_ms,diag_move_blend,diag_pos_bias,
                       diag_pos_i_trim,diag_k2_eff,diag_k1_eff,
                       diag_alpha_dot_raw,diag_theta_dot_raw,diag_alpha_raw,diag_theta_raw,
                       diag_theta_e,diag_term_k4,diag_term_k3,diag_term_k2,diag_term_k1,
                       8'd0,alpha_trim,{7'd0,diag_obs_active},diag_duty_pre_dz,
                       diag_k4_eff,diag_k3_eff,diag_alpha_pre,diag_pos_trim,
                       7'd0,run_en,7'd0,calib_done,offset_c01,adc,encoder,
                       target,reference,duty,theta_dot,theta,alpha_dot,alpha,
                       {6'd0,otr_fault,fault},{5'd0,mode},revision,tick_count};
            snap_ready<=1;telemetry_seq<=telemetry_seq+1'b1;
        end
    endtask
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            // 上限用宏：曾硬编码 10/11 漏掉新增 id（id10 甩摆距离、id11/12 POS 对）
            for(i=0;i<`DEBUG_PARAM_COUNT;i=i+1)begin active[i]<=default_value(i);shadow[i]<=default_value(i);end
            transaction<=0;commit_pending<=0;txn_expired<=0;txn_age<=0;session<=0;tick_count<=0;
            revision<=0;last_seq<=0;last_kind<=0;last_length<=0;last_request<=0;cached_reply<=0;
            reply<=0;reply_seq<=0;cached_length<=0;reply_length<=0;reply_ready<=0;have_last<=0;
            pending_seq<=0;pending_kind<=0;pending_request<=0;pending_length<=0;
            telemetry_seq<=0;period<=0;snapshot<=0;snap_ready<=0;dropped<=0;
            send<=0;send_kind<=0;send_seq<=0;send_length<=0;send_payload<=0;
            remote_start<=0;remote_stop<=0;remote_clear<=0;remote_mode<=0;remote_calib<=0;
            move_pending<=0;move_delta<=0;io_test_enable<=0;io_test_duty<=0;
            move_queued<=0;move_q_delta<=0;move_q_is_abs<=1'b0;move_is_abs<=1'b0;move_hold<=8'd0;remote_mode_hold<=8'd0;
            force_hold<=8'd0;remote_force_pos<=1'b0;
            hil_tick<=0;tick_d<=0;hil_adc<=0;hil_enc<=0;
            step_pending<=0;step_wait<=0;step_seq<=0;step_len<=0;step_req<=0;
        end else begin
            send<=0;
            hil_tick<=0;         // STEP 产生单拍 tick 脉冲
            tick_d<=tick;
            // HIL: 每个 STEP 都回一帧遥测，主机用它推进本地被控对象；
            // STEP ACK 另外携带本拍 duty/mode/fault，供锁步确认。
            if(tick_d && hil_mode)take_snapshot;
            // STEP 应答要报告**本拍算完**的 duty/tick，而 LQR(3 级)+duty_sel+
            // out_stage 需要若干时钟才稳定，故推迟 8 拍再应答。
            if(step_pending)begin
                if(step_wait==0)begin
                    step_pending<=0;
                    // STEP 应答载荷（紧随 14 字节公共前缀之后）：
                    //   +0 tick u32, +4 duty s16, +6 mode u8, +7 fault u8
                    // duty 取自 out_stage，此刻已按本拍完成压摆更新；mode/fault
                    // 供上位机判定是否施力，必须与 duty 同拍，故整帧一并回传。
                    answer=0;
                    answer[143:112]=tick_count;
                    answer[159:144]=duty;
                    answer[167:160]={5'd0,mode};
                    answer[175:168]={7'd0,fault};
                    complete_reply(STEP,OK,step_seq,step_len,step_req,answer,16'd22,
                                   revision,tick_count,session);
                end else step_wait<=step_wait-1'b1;
            end
            if(tick)begin
                tick_count<=tick_count+1;
                remote_start<=0;remote_stop<=0;remote_clear<=0;remote_mode<=0;remote_calib<=0;
                // remote_mode 保持（2026-11-03 NO_POS 定谳）：MOVE 排队执行器只在
                // move_exec_q 那一拍发 remote_mode=1 单拍脉冲，而 mode_fsm 的
                // BAL→POS 门 key_mode&&pos_ready 要求【同拍】α±5°/α̇±30°/θ̇ 三门
                // 全满足——1.4% 通过率×1 拍 ≈ 哑火（151537/155021 六轮全 NO_POS）。
                // 与 move_hold 同步保持 200 拍，让 mode_fsm 有窗口等到 pos_ready。
                if(remote_mode_hold!=8'd0) begin
                    remote_mode_hold <= remote_mode_hold - 8'd1;
                    remote_mode <= 1'b1;
                end
                // 强制进 POS 的旁路标志：与 remote_mode 同步保持 200 拍，
                // 让 mode_fsm 在 BAL 稳态下任一拍都能立刻切 M_POS。
                if(force_hold!=8'd0) begin
                    force_hold <= force_hold - 8'd1;
                    remote_force_pos <= 1'b1;
                end else remote_force_pos <= 1'b0;
                // 定速旋转使能只在本轮 POS 内有效：离开 POS 立即清零（2026-11-04，
                // 治"停止按钮没用/进 POS 自己就转"——原使能位锁存，STOP 不清）。
                if (mode != 3'd3) vel_demo_en <= 1'b0;
                // move_pending 保持 move_hold 个 tick 再清（原先 mode==3||safe 时
                // 直接清零，POS 内的 MOVE 根本来不及被 ctrl_core 采样）。BALANCE
                // 场景仍按原语义很快清掉，不影响 MOVE→POS 切换时序。
                if(move_hold!=8'd0) begin
                    move_hold <= move_hold - 8'd1;
                end else if(mode==3 || safe) begin
                    move_pending<=0;
                end
                // ---- 排队目标执行器 ----
                // 受理后挂在这里，门一开就发脉冲；离开 BALANCE/POS 即丢弃
                // （不补执行掉摆/急停前积压的定位意图）。必须放在上方
                // move_hold/move_pending 清零逻辑**之后**：否则同拍
                // move_pending<=1 会被 else-if 的 move_pending<=0 覆盖，
                // 排队执行永远无效（champion 10-25 板上实测踩过）。
                if(move_queued && mode!=2 && mode!=3) begin
                    move_queued<=0;
                end else if(move_queued && move_exec_q) begin
                    if(mode==2)begin remote_mode<=1; remote_mode_hold<=8'd200; end
                    move_pending<=1;
                    move_hold<=8'd200;
                    move_delta<=move_q_delta;
                    move_is_abs<=move_q_is_abs;
                    move_q_is_abs<=1'b0;
                    move_queued<=0;
                end
                period<=~period;
                if(period && !hil_mode)take_snapshot;
                if(transaction)begin
                    if(!safe || txn_age==4999 || local_stop || local_start)begin
                        transaction<=0;txn_expired<=1;
                    end else txn_age<=txn_age+1'b1;
                end
                if(commit_pending)begin
                    commit_pending<=0;transaction<=0;
                    if(safe && !local_start && !local_stop)begin
                        for(i=0;i<`DEBUG_PARAM_COUNT;i=i+1)active[i]<=shadow[i];
                        revision<=revision+1;
                        complete_reply(pending_kind,OK,pending_seq,pending_length,pending_request,
                                       512'd0,16'd14,revision+1,tick_count+1,session);
                    end else complete_reply(pending_kind,STATE,pending_seq,pending_length,pending_request,
                                            512'd0,16'd14,revision,tick_count+1,session);
                end
            end
            // ACK has priority; acceptance handshake includes send itself.
            if(!sender_busy && !send)begin
                if(reply_ready)begin
                    send<=1;send_kind<=8'h80;send_seq<=reply_seq;send_length<=reply_length;
                    send_payload<=reply;reply_ready<=0;
                end else if(snap_ready)begin
                    send<=1;send_kind<=8'h90;send_seq<=telemetry_seq;send_length<=SNAPSHOT_BYTES;
                    send_payload<=snapshot;snap_ready<=0;  // v5 = 80B 满帧，无填充
                end
            end
            // Host uses stop-and-wait. Frames arriving while reply waits are
            // ignored; host retransmits same sequence with no repeated action.
            if(frame_valid && !reply_ready && !commit_pending)begin
                code=OK;answer=0;answer_length=14;
                if(have_last && frame_seq==last_seq && frame_kind==last_kind &&
                   frame_length==last_length && duplicate_payload)begin
                    reply<=cached_reply;reply_length<=cached_length;reply_seq<=last_seq;reply_ready<=1;
                end else if(frame_kind==HELLO && frame_seq==0)begin
                    if(frame_length!=8)code=BAD_LENGTH;
                    else if(frame_payload[31:0]!=`DEBUG_SCHEMA || frame_payload[63:32]==0)code=BAD_SCHEMA;
                    if(code==OK)begin
                        session<=frame_payload[63:32];transaction<=0;txn_expired<=0;
                        answer[143:112]=`DEBUG_SCHEMA;answer[175:144]=`DEBUG_BUILD_STAMP;
                        // 能力位随 hil_mode 切换：真实传感模式不接受 STEP，
                        // 锁步模式下没有真实硬件传感/电机（位定义见文件头）。
                        answer[207:176]=hil_mode ? `DEBUG_CAPS_HIL : `DEBUG_CAPS_REAL;
                        answer_length=26;
                    end
                    complete_reply(frame_kind,code,frame_seq,frame_length,frame_payload,answer,
                                   answer_length,revision,tick_count,code==OK ? frame_payload[63:32] : session);
                end else if(session==0 || frame_length<4 || frame_payload[31:0]!=session ||
                            frame_seq!=last_seq+16'd1)begin
                    // Invalid session/sequence never replaces duplicate cache.
                    reply<=0;reply[7:0]<=frame_kind;reply[15:8]<=SEQUENCE;
                    reply[47:16]<=revision;reply[79:48]<=tick_count;reply[111:80]<=session;
                    reply_seq<=frame_seq;reply_length<=14;reply_ready<=1;
                end else begin
                    case(frame_kind)
                        READ:begin
                            if(frame_length!=6)code=BAD_LENGTH;
                            // 上限用宏：曾硬编码 10 漏掉 id10(fling_delta)，设备误拒
                            // BAD_ID 致上位机全量读参即断连（2026-09-22 实测）
                            else if(param_id>=`DEBUG_PARAM_COUNT)code=BAD_ID;
                            else begin answer[143:112]={{16{active[param_id][15]}},active[param_id]};answer_length=18;end
                        end
                        BEGIN_TXN:begin
                            if(frame_length!=8)code=BAD_LENGTH;
                            else if(!safe || remote_start || local_start)code=STATE;
                            else if(frame_payload[63:32]!=revision)code=CONFLICT;
                            else begin
                                for(i=0;i<`DEBUG_PARAM_COUNT;i=i+1)shadow[i]<=active[i];
                                transaction<=1;txn_age<=0;txn_expired<=0;
                            end
                        end
                        SET:begin
                            if(frame_length!=10)code=BAD_LENGTH;
                            else if(!safe)code=STATE;
                            else if(!transaction)code=txn_expired ? EXPIRED : NO_TXN;
                            else if(param_id>=`DEBUG_PARAM_COUNT)code=BAD_ID;
                            else if(!valid_value(param_id,param_value))code=RANGE;
                            else begin shadow[param_id]<=param_value[15:0];txn_age<=0;end
                        end
                        COMMIT:begin
                            if(frame_length!=8)code=BAD_LENGTH;
                            else if(!safe || remote_start || local_start)code=STATE;
                            else if(!transaction)code=txn_expired ? EXPIRED : NO_TXN;
                            else if(frame_payload[63:32]!=revision)code=CONFLICT;
                            else if(shadow[8]<=shadow[7])code=RANGE;
                            else begin
                                commit_pending<=1;pending_seq<=frame_seq;pending_kind<=frame_kind;
                                pending_request<=frame_payload;pending_length<=frame_length;
                            end
                        end
                        ABORT:if(frame_length!=4)code=BAD_LENGTH;
                              else begin transaction<=0;txn_expired<=0;end
                        DEFAULTS:begin
                            if(frame_length!=4)code=BAD_LENGTH;
                            else if(!safe)code=STATE;
                            else if(!transaction)code=NO_TXN;
                            else for(i=0;i<`DEBUG_PARAM_COUNT;i=i+1)shadow[i]<=default_value(i);
                        end
                        STOP:if(frame_length!=4)code=BAD_LENGTH;
                             else begin remote_stop<=1;transaction<=0;move_pending<=0;
                                 io_test_enable<=0;io_test_duty<=0;end
                        CLEAR:if(frame_length!=4)code=BAD_LENGTH;
                              else if(!safe || otr_fault)code=STATE;
                              else begin remote_clear<=1;io_test_enable<=0;io_test_duty<=0;end
                        START:if(frame_length!=4)code=BAD_LENGTH;
                              else if(mode!=0 || fault || config_lock || local_stop || local_start)code=STATE;
                              else remote_start<=1;
                        MOVE:begin
                            if(frame_length!=8)code=BAD_LENGTH;
                            // 2026-09-29：θ̇ 门 200→600，与 mode_fsm pos_ready 同步放宽。
                            // 2026-09-30：加臂慢速条件（stall 或 <15°/s）——稳摆本身
                            // 要求臂持续微调（~6°/s 匀漂永不停），纯 stall 门永不打开；
                            // 两道门条件必须一致，否则 clk/tick 采样错开会出现
                            // "受理了却不切换"。α ±500 不动（摆杆须近垂直）。
                            // 2026-11-01 P1（移植 champion 2026-10-27）：状态门不再
                            // 拒绝命令——只决定执行时机（move_exec 排队执行器）。硬拒绝
                            // 只留故障/配置锁/急停；停摆/α 超窗一律**排队**（P0 板测
                            // 首发 0/3 全被拒=上位机只能碰运气）。已在 POS 时不拉
                            // remote_mode——mode_fsm 的 M_POS 分支把 key_mode 当"按
                            // 模式键"处理，会弹回 BALANCE（champion 10-21 实测教训）。
                            else if(fault || config_lock || local_stop)code=STATE;
                            else if(requested_delta< -6000 || requested_delta>6000 ||
                                    requested_end>33'sd2147483647 || requested_end< -33'sd2147483648)code=RANGE;
                            else begin
                                move_q_delta <= requested_delta;
                                move_queued  <= 1'b1;
                            end
                        end
                        MOVE_ABS:begin
                            // 绝对目标定位（2026-11-04）：payload = target c01（0-360 域
                            // 或多圈均可，固件按当下 θ 取最短路）。用于替代"相对 MOVE"，
                            // 消除上位机数据陈旧造成的终点偏差。
                            if(frame_length!=8)code=BAD_LENGTH;
                            else if(fault || config_lock || local_stop)code=STATE;
                            else if(abs_delta < -33'sd33000 || abs_delta > 33'sd33000)code=RANGE;
                            else begin
                                // 存【绝对目标】+ 标志：执行拍由 ctrl_core 直接置
                                // pos_tgt_cmd = 目标，避免"到达时算 delta、执行时再加 θ"
                                // 之间的臂位移误差（2026-11-04 实测差 20°）。
                                move_q_delta  <= requested_delta;
                                move_q_is_abs <= 1'b1;
                                move_queued   <= 1'b1;
                            end
                        end
                        FORCE_POS:begin
                            // 2026-11-03 夜（用户需求）：BAL 态强制切 POS。
                            // MOVE 的 BAL→POS 走 mode_fsm 的 pos_ready 门（|α|<5°、
                            // |α̇|<30°/s、臂<40°/s），稳摆摆动下常等不到；本命令
                            // 只保留 |α|<15° 的安全界，入场瞬态交给 1.5s 稳定窗。
                            if(frame_length!=8)code=BAD_LENGTH;
                            else if(fault || config_lock || local_stop)code=STATE;
                            else if(mode!=3'd2)code=STATE;
                            else if(alpha>16'sd1500 || alpha<-16'sd1500)code=STATE;
                            else if(requested_delta< -6000 || requested_delta>6000)code=RANGE;
                            else begin
                                move_q_delta <= requested_delta;
                                move_queued  <= 1'b1;
                                remote_mode  <= 1'b1;
                                remote_mode_hold <= 8'd200;
                                remote_force_pos <= 1'b1;
                                force_hold <= 8'd200;
                            end
                        end
                        VEL_DEMO:begin
                            // 2026-11-03 夜：速度曲线演示。载荷 session4 + en u16 +
                            // V s16(0.1°/s) + A s16(0.1°/s per ms)，共 10 字节。
                            // 只允许 POS（mode 3）内开启；开启后固件内生三角波 v_ref，
                            // LQR 的 θ̇ 项跟踪 (θ̇−v_ref)，pos_tgt 跟随臂（K3≈0）。
                            if(frame_length!=10)code=BAD_LENGTH;
                            else if(fault || config_lock || local_stop)code=STATE;
                            else if(frame_payload[47:32]>16'd1)code=RANGE;
                            else if(frame_payload[47:32]==16'd1 && mode!=3'd3)code=STATE;
                            else if($signed(frame_payload[63:48])<16'sd0 ||
                                    $signed(frame_payload[63:48])>16'sd800)code=RANGE;
                            else if($signed(frame_payload[79:64])<16'sd1 ||
                                    $signed(frame_payload[79:64])>16'sd300)code=RANGE;
                            else begin
                                vel_demo_en <= frame_payload[47:32]!=16'd0;
                                vel_demo_v  <= $signed(frame_payload[63:48]);
                                vel_demo_a  <= $signed(frame_payload[79:64]);
                            end
                        end
                        BALANCE:begin
                            if(frame_length!=4)code=BAD_LENGTH;
                            else if(fault || local_stop || local_start || config_lock)code=STATE;
                            // 原行为：POS -> BALANCE。新增调参入口：手扶直立并静止时
                            // IDLE -> BALANCE，直接接管 LQR，不经过 SWING 的甩摆输出。
                            else if(mode==3)remote_mode<=1;
                            else if(mode==0 && alpha>-16'sd500 && alpha<16'sd500 &&
                                    alpha_dot>-16'sd300 && alpha_dot<16'sd300 &&
                                    theta_dot>-16'sd200 && theta_dot<16'sd200)
                                remote_mode<=1;
                            else code=STATE;
                        end
                        CALIBRATE:if(frame_length!=4)code=BAD_LENGTH;
                                  else if(!safe || config_lock)code=STATE;else remote_calib<=1;
                        STATUS:begin
                            if(frame_length!=4)code=BAD_LENGTH;
                            else begin
                                answer[143:112]=rx_errors;
                                answer[175:144]=dropped;
                                answer[207:176]=diag_encoder;
                                answer[223:208]=diag_adc;
                                answer[239:224]=io_test_duty;
                                answer[247:240]={7'd0,io_test_enable};
                                // MT6701 链路健康（2026-10-19，向后兼容追加）：
                                // 旧上位机只解析前 12 字节，多出的尾巴被忽略。
                                answer[263:248]=mt6_frames;
                                answer[279:264]=mt6_good;
                                answer[295:280]=mt6_crc_err;
                                answer[311:296]=mt6_mg_err;
                                answer[319:312]={4'd0,mt6_mg_code};
                                answer[327:320]={mt6_cause,4'd0,mt6_sel,mt6_fault};
                                answer[343:328]={2'd0,mt6_angle_raw};
                                // MT6835 链路健康（2026-10-19 二次追加）：旧上位机
                                // 按 12/14 字节前缀截断，多出的尾巴被忽略。
                                answer[359:344]=mt68_frames;
                                answer[375:360]=mt68_good;
                                answer[391:376]=mt68_crc_err;
                                answer[407:392]=mt68_status_err;
                                answer[415:408]={5'd0,mt68_status};
                                answer[423:416]={7'd0,mt68_fault};
                                answer[455:424]={11'd0,mt68_angle_raw};
                                // 最长无好帧段（2026-10-20 12:40 cause=0 揭示看门狗
                                // 路径在发火；此字段量出扰动段真实长度）
                                answer[471:456]=mt6_max_no_good;
                                answer_length=59;
                            end
                        end
                        IO_TEST:begin
                            // 载荷：session u32 + duty s16 + enable u16。
                            // 只允许安全态、±20% 以内的低速点动，防止误启动电机。
                            if(frame_length!=8)code=BAD_LENGTH;
                            else if(!safe)code=STATE;
                            else if(frame_payload[63:48]>16'd1 ||
                                    $signed(frame_payload[47:32]) < -16'sd6000 ||
                                    $signed(frame_payload[47:32]) > 16'sd6000)code=RANGE;
                            else begin
                                io_test_duty<=$signed(frame_payload[47:32]);
                                io_test_enable<=frame_payload[63:48]!=16'd0;
                            end
                        end
                        STEP:begin
                            // HIL: 锁存上位机传感器字并推进恰好一个控制毫秒。
                            // 本模块的 hil_tick 单拍脉冲驱动 ctrl_core 的 tick；
                            // sensor 输入随之切换为注入源。应答推迟到控制链稳定，
                            // 携带本拍 duty 供上位机积分被控对象。
                            // 受理成功时 code 保持 OK，下面的应答分发据
                            // "frame_kind==STEP && code==OK" 判定本帧已转交
                            // step_pending 处理，不再立即应答。
                            if(!hil_mode)code=STATE;
                            else if(frame_length!=10)code=BAD_LENGTH;
                            else if(step_pending)code=STATE;  // 上一拍未应答完
                            else begin
                                hil_adc<=frame_payload[47:32];
                                hil_enc<=$signed(frame_payload[79:48]);
                                hil_tick<=1;
                                step_pending<=1;step_wait<=4'd8;
                                step_seq<=frame_seq;step_len<=frame_length;
                                step_req<=frame_payload;
                            end
                        end
                        default:code=BAD_ID;
                    endcase
                    // STEP 受理成功时其应答由上面的 step_pending 分支发出（需等控制链
                    // 稳定才有正确 duty），此处不得重复应答。
                    if((frame_kind!=COMMIT || code!=OK) &&
                       !(frame_kind==STEP && code==OK))
                        complete_reply(frame_kind,code,frame_seq,frame_length,frame_payload,answer,
                                       answer_length,revision,tick_count,session);
                end
            end
        end
    end
endmodule
