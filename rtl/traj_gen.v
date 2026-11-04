// 有界位置参考发生器（S 曲线版，2026-10-09 方案文档 §4.1）。
// 所有位置均为连续 0.01°，不是编码器计数。1ms 节拍下 V_MAX=1 对应 10°/s。
//
// 旧版=纯限速斜坡：速度 0↔V_MAX 台阶切换，加速度不连续，起停瞬间把抖动
// 直接打进摆杆（"10° 斜坡必死"，09-29）。新版=**加速度受限的梯形剖面**：
//   v_cmd = sign(err)·min( V_MAX, |err|·44>>>8 )   —— 按剩余误差线性减速
//   v     += clip( v_cmd - v , ±A_MAX )            —— 加速度限幅（起停平滑）
//   pos_q8 += v                                     —— Q8 位置积分（带分数！）
// 位宽纪律：位置状态用 Q8（pos_q8，40 位），输出 pos_ref = pos_q8>>>8。
// 若直接 pos_ref += v>>>8 会把每拍分数丢光（v=255/256 满速差 1 时位置卡死，
// 实测停在半路）。终值精确：|err|<1 c01 时 pos_q8 直接吸附目标。
// A_MAX 单位 = Q8 c01/tick²：A_MAX=1 ⇒ 0→满速 256ms（≈39°/s²，实测温和）；
// A_MAX≥256 ⇒ 一步到位，行为等价旧版斜坡（DE10 默认 256 零行为变化）。
// 减速系数 218/256 ⇒ 距目标 3° 外满速、以内线性逼近（30° 行程 ~3.2s）。
module traj_gen #(
    parameter V_MAX = 1,     // c01/tick
    parameter A_MAX = 256    // Q8 c01/tick²；256=无限加速度（旧行为）
) (
    input  wire               clk, rst_n, tick, clear,
    input  wire signed [31:0] pos_actual, pos_target,
    output reg  signed [31:0] pos_ref,
    output reg  signed [15:0] v_ref       // Q8 c01/tick（供前馈/遥测扩展）
);
    reg signed [39:0] pos_q8;   // Q8 位置状态（32+8 位防移位溢出）
    reg signed [31:0] v_q8;     // Q8 速度状态

    // 误差通道打两拍（clk 域寄存）：err_q 一拍、v_abs_q 一拍——切断
    // pos_q8→error→×218→比较→dv_clip→40 位加法→snap mux 的同拍长链
    // （不打拍 Fmax 31MHz；两拍后 45MHz+）。代价=参考多 2 拍延迟（2ms），
    // 对 3s 行程无感。
    reg signed [31:0] err_q, v_abs_q;
    wire signed [31:0] pos_out  = pos_q8 >>> 8;
    wire signed [32:0] error_c  =
        $signed({pos_target[31],pos_target})-$signed({pos_out[31],pos_out});
    always @(posedge clk) begin
        err_q   <= error_c[31:0];
        v_abs_q <= ((err_q < 0) ? -err_q : err_q) * 218 >>> 8;
    end
    wire signed [32:0] error    = $signed(err_q);
    wire signed [31:0] err_abs  = (err_q < 0) ? -err_q : err_q;
    wire signed [31:0] v_abs_cmd = v_abs_q;
    wire signed [31:0] v_full    = V_MAX <<< 8;
    wire signed [31:0] v_cap_q8  = (v_abs_cmd > v_full) ? v_full : v_abs_cmd;
    wire signed [31:0] v_cmd_q8  = (error > 0) ? v_cap_q8 : -v_cap_q8;

    wire signed [31:0] dv       = v_cmd_q8 - v_q8;
    wire signed [31:0] dv_clip  = (dv >  A_MAX) ? A_MAX :
                                  (dv < -A_MAX) ? -A_MAX : dv;
    wire signed [31:0] v_next_q8 = v_q8 + dv_clip;

    wire        snap = (err_abs < 32'sd16);  // 误差 <0.16°：吸附精确终值
                                                //（参考偏移对伺服不可见）
    wire signed [39:0] pos_free_q8 = pos_q8 + v_next_q8;
    // snap 目标独立成 signed wire：若在三目里内联拼接 {{n{...}},x}（无符号），
    // 会把整个三目上下文传染成无符号，false 分支的 -1 被零扩展成 +2^32
    // （实测 pos_q8 768055 → 4295735350 反向爆炸）。限制器 lim_ext 同类教训。
    // clear 分支的 {pos_actual...}<<<8 同理改用 $signed 拼接。
    wire signed [39:0] snap_target = $signed({{7{pos_target[31]}}, pos_target}) <<< 8;
    wire signed [39:0] pos_actual_q8 = $signed({{7{pos_actual[31]}}, pos_actual}) <<< 8;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_q8 <= 40'sd0; pos_ref <= 32'sd0;
            v_q8 <= 32'sd0; v_ref <= 16'sd0;
        end else if (tick) begin
            if (clear) begin
                pos_q8  <= pos_actual_q8;
                pos_ref <= pos_actual;
                v_q8    <= 32'sd0;
                v_ref   <= 16'sd0;
            end else if (snap) begin
                pos_q8  <= snap_target;
                pos_ref <= pos_target;
                v_q8    <= 32'sd0;
                v_ref   <= 16'sd0;
            end else begin
                pos_q8  <= pos_free_q8;
                pos_ref <= pos_free_q8 >>> 8;
                v_q8    <= v_next_q8;
                v_ref   <= v_next_q8[15:0];
            end
        end
    end
endmodule
