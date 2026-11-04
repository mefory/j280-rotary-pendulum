"""1ms控制器 + 0.1ms RK4 Furuta对象。用于无板交互；正式验收仍使用SLX/RTL。"""
import json
import math
import random
from pathlib import Path
from .schema import ROOT


def clip(x, limit):
    return min(max(x, -limit), limit)


def wrap(x):
    x=(x+18000)%36000-18000
    return 18000 if x==-18000 else x


# 摆臂测速（与 rtl/arm_vel.v 的发布参数逐条对应；改这里必须同步改 RTL 参数）
USE_T = True               # 低速段用 T 法（0=纯 M 法，即旧发布行为）
EMA_SH = 2                 # θ̇ 的 EMA 移位 2^n（RTL 同名参数）
IQ_SH = 11                 # 定位积分 trim 增益 2^-n（duty per cdeg·tick）；限幅 ±600 duty
STALL_TICKS = 300          # 无边沿判停阈值（拍）；臂停下 300ms 内 θ̇ 归零
VNUM_T = 3461              # 10*360*1000/CPR（CPR=1040）：1 计数/1 拍 ⇒ 3461（0.1°/s）


class Plant:
    def __init__(self, seed=7, profile=None):
        bundle=json.loads((ROOT/"config/plant.json").read_text(encoding="utf-8"))
        self.p=bundle["parameters"].copy()
        sensor={}
        if profile is not None:
            identified=json.loads(Path(profile).read_text(encoding="utf-8"))
            self.p.update(identified["parameters"])
            sensor=identified.get("sensor", {})
        self.p.setdefault("motor_gain", 1.0)
        self.sensor_phase_deg=sensor.get("phase_deg", 263.44)
        self.initial_offset=sensor.get("offset_c01", 8344)
        p=self.p
        self.j0=p["J_arm"]+p["m_rod"]*p["L_arm"]**2
        self.jp=p["J_rod"]+p["m_rod"]*p["lc_rod"]**2
        self.c=p["m_rod"]*p["L_arm"]*p["lc_rod"]
        self.mg=p["m_rod"]*p["g"]*p["lc_rod"]
        self.seed=seed
        self.reset()

    def reset(self):
        self.random=random.Random(self.seed)
        self.x=(0.,math.pi,0.,0.)
        self.mode=4;self.fault=1;self.ticks=0
        self.reference=0;self.target=0;self.iq=0;self.duty=0
        self.alpha=18000;self.alpha_dot=0;self.theta=0;self.theta_dot=0
        self.ah=[18000]*4;self.ch=[0]*4;self.d4=0;self.aq=0;self.tq=0
        self.tv_pos=0;self.tv_edge=0;self.tv_p=4095;self.tv_dir=1
        self.adc=15835;self.encoder=0;self.sensor=(15835,0)
        self.offset=self.initial_offset;self.calib_done=0;self.calib_samples=[]
        self.calibrating=False;self.warm=0;self.prev_mode=4;self.fling_tgt=0
        self.swing_ticks=0;self.catch_ms=0;self.catch_boost=False

    def rhs(self, x, tau):
        q,a,v,w=x;s=math.sin(a);co=math.cos(a)
        m11=self.j0+self.jp*s*s;m12=self.c*co
        r1=tau-2*self.jp*s*co*v*w+self.c*s*w*w-self.p["b_vis"]*v
        r2=self.mg*s+self.jp*s*co*v*v-self.p["b_rod"]*w
        d=m11*self.jp-m12*m12
        return v,w,(self.jp*r1-m12*r2)/d,(-m12*r1+m11*r2)/d

    def integrate(self, tau):
        h=.0001
        for _ in range(10):
            x=self.x
            a=self.rhs(x,tau)
            b=self.rhs(tuple(z+h*k/2 for z,k in zip(x,a)),tau)
            c=self.rhs(tuple(z+h*k/2 for z,k in zip(x,b)),tau)
            d=self.rhs(tuple(z+h*k for z,k in zip(x,c)),tau)
            self.x=tuple(z+h*(ka+2*kb+2*kc+kd)/6 for z,ka,kb,kc,kd in zip(x,a,b,c,d))
        if not all(math.isfinite(v) for v in self.x):
            raise ArithmeticError("Nonfinite simulated state")

    # ---- 锁步 HIL 接口（与 step() 共用同一套量化/物理，避免两套实现走样）----

    def sample_sensors(self):
        """按当前状态生成传感器原始字，不推进时间。

        与 step() 开头完全同口径：摆角经所选模型传感器零偏 + u16 满量程 65472 量化，
        加 1.7LSB 高斯噪声；摆臂角按 1040 计数/圈量化。HIL 下由本函数提供
        FPGA 的输入，FPGA 内部控制链自带采样延迟，故此处不再叠加延迟。
        """
        raw_angle=min((self.x[1]*180/math.pi+self.sensor_phase_deg)%360,345)
        adc=int(min(65472,max(0,math.floor(raw_angle/345*65472+self.random.gauss(0,1.7)+.5))))
        counts=self.x[0]/(2*math.pi)*1040
        enc=math.floor(counts+.5) if counts>=0 else math.ceil(counts-.5)
        self.adc=adc;self.encoder=enc
        return adc,enc

    def torque_for(self, duty, mode, fault=0):
        """duty → 轴力矩：含 ±350 死区扣除、方向保持与 Vbus/Kt/R 换算。

        与 step() 末尾 same 口径；mode 取 FPGA 遥测值，非驱动模式一律零力矩。
        """
        effective=duty if mode in (1,2,3) and not fault else 0
        return (math.copysign(max(abs(effective)-350,0),effective)*self.p["Vbus"]/10000*
                self.p["Kt"]/self.p["R"]*self.p["motor_gain"])

    def physics_step(self, duty, mode, fault=0):
        """锁步一拍：积分物理，控制器由 FPGA 提供。"""
        self.ticks+=1
        self.duty=duty
        self.integrate(self.torque_for(duty,mode,fault))
        return self.sample_sensors()

    def _arm_speed(self, cnt):
        """摆臂角速度：M/T 混合 + Q8 EMA，与 rtl/arm_vel.v（USE_T=1）逐条同口径。

        M 法（4ms 窗差分）高速段精度好，但低速每计数台阶 865（86.5°/s）。实机证据
        （2026-09-23）：稳摆时摆臂被齿隙晃动 ±1~3 计数，M 法读成 ±865~2600，
        经 K4（1.607 duty/单位）放大成 ±9000 duty 的输出抖动（duty 峰峰 14770、
        换向 34.5 次/s、机械咔咔共振）⇒ 低速段必须用 T 法（量相邻计数边沿间隔）。

        T 法除数取 `max(上次边沿周期, 距上次边沿的实时拍数)`：匀速时二者相等（估计
        有效），减速/停下时实时时间变大 ⇒ 估计平滑衰减，不会在停车后挂着陈旧高速值。
        无边沿 STALL_TICKS 拍后硬归零；|4 拍差分| ≥ 2 计数（≥173°/s）改走 M（高速段，
        含起摆捕获——捕获需要 K4 对高速摆臂的阻尼，K4 不可砍，2026-09-23 实机验证）。
        """
        dp = cnt - self.ch[3]
        self.ch = [cnt] + self.ch[:3]
        v_m = clip(dp * 1731 // 2, 32767)          # M_SCALE: 0.1°/s per count per 4ms
        if cnt != self.tv_pos:                     # 计数边沿：锁存周期与方向
            self.tv_p = min(4095, max(1, self.ticks - self.tv_edge))
            self.tv_edge = self.ticks
            self.tv_dir = 1 if cnt > self.tv_pos else -1
            self.tv_pos = cnt
        elapsed = self.ticks - self.tv_edge
        if elapsed >= STALL_TICKS:
            v_t = 0                                # 判停
        else:
            den = max(self.tv_p, elapsed)          # 取较大者 ⇒ 停车后平滑衰减
            v_t = self.tv_dir * min(32767, VNUM_T // den)
        v = (v_m if abs(dp) >= 2 else v_t) if USE_T else v_m
        self.tq += (clip(v, 32767) * 256 - self.tq) // (1 << EMA_SH)
        return int(clip((self.tq + 128) // 256, 32767))

    def step(self, values):
        self.ticks+=1
        raw_angle=min((self.x[1]*180/math.pi+self.sensor_phase_deg)%360,345)
        self.adc=int(min(65472,max(0,math.floor(raw_angle/345*65472+self.random.gauss(0,1.7)+.5))))
        counts=self.x[0]/(2*math.pi)*1040
        self.encoder=math.floor(counts+.5) if counts>=0 else math.ceil(counts-.5)
        adc,cnt=self.sensor;self.sensor=(self.adc,self.encoder)
        raw=adc*34534//65536
        self.alpha=wrap(raw-self.offset+18000)
        self.theta=cnt*35446//1024
        if self.calibrating:
            self.calib_samples.append(raw)
            if len(self.calib_samples)==2000:
                self.offset=wrap(round(sum(self.calib_samples)/2000))
                self.calib_done=1;self.calibrating=False;self.warm=0
        if self.warm<2:
            self.ah=[self.alpha]*4;self.ch=[cnt]*4;self.warm+=1
            self.reference=self.theta;self.target=self.theta;self.duty=0
        else:
            dr=wrap(self.alpha-self.ah[3]);self.ah=[self.alpha]+self.ah[:3]
            self.aq+=(clip(25*self.d4,32767)*256-self.aq)//4
            self.alpha_dot=int(clip((self.aq+128)//256,32767));self.d4=dr
            self.theta_dot=self._arm_speed(cnt)
            if self.mode==1:
                self.swing_ticks+=1
                if self.swing_ticks>=25000:
                    self.mode=4;self.reference=self.theta;self.target=self.theta
                elif abs(self.alpha)<values[7] and abs(self.theta_dot)<5000:
                    self.mode=2;self.reference=self.theta;self.target=self.theta;self.swing_ticks=0
            elif self.mode in (2,3) and abs(self.alpha)>values[8]:
                self.mode=4;self.reference=self.theta;self.target=self.theta
            else:
                self.swing_ticks=0
            if self.mode==3:
                self.reference+=int(clip(self.target-self.reference,1))
                # 积分 trim（与 ctrl_core.v 同口径）：门保持原样（斜坡完成+近目标+摆杆
                # 直立+θ̇ 小+输出未饱和），但增益 2^20→2^11、限幅 ±200→±600。
                # 旧增益下 E=200cdeg 累积 1000 拍只出 0.2 duty，而死区 350 ⇒ 从未起过作用；
                # 末段逼近原本靠 M 法 θ̇ 噪声经 K4 的"抖动整流"偶然顶过死区（实机上同一
                # 机制被放大成 ±9000 duty 的暴力共振）。T 法洗净 θ̇ 后必须由积分器
                # 确定性地补上这一步：300cdeg 残差 ~0.7s 内推出 ±350 死区。
                if (self.reference==self.target and abs(self.theta-self.reference)<=500 and
                    abs(self.alpha)<500 and abs(self.theta_dot)<200 and abs(self.duty)<9000):
                    self.iq=int(clip(self.iq+self.theta-self.reference,600<<IQ_SH))
            else:
                self.iq=0
            if self.mode==1:
                # 甩摆起摆（与 swingup_core 同口径）：进入沿锁存 theta0+Δ，位置伺服甩臂
                if self.prev_mode!=1:self.fling_tgt=self.theta
                u=int(clip(9*(self.fling_tgt+values[10]-self.theta),values[6]))
            elif self.mode in (2,3):
                # 捕获增强相（与 ctrl_core.v 同口径）：SWING→BALANCE 后 800ms 内
                # K2/K4 提 13/8 倍，压住 ~480°/s 入窗动量
                if self.prev_mode==1 and self.mode==2:
                    self.catch_ms=0;self.catch_boost=True
                elif self.catch_boost:
                    if self.mode!=2 or self.catch_ms>=800:self.catch_boost=False
                    else:self.catch_ms+=1
                g=[values[0],values[1],values[2],values[3]]
                if self.catch_boost:
                    g[1]=(1 if g[1]>=0 else -1)*min(abs(g[1])*8>>3,30000)
                    g[3]=(1 if g[3]>=0 else -1)*min(abs(g[3])*8>>3,30000)
                if self.mode==3:
                    # POS 专用臂环对（与 ctrl_core.v 的 mux 同口径）：运行时 id11/id12
                    g[2]=values[11];g[3]=values[12]
                state=(self.alpha,self.alpha_dot,clip(self.theta-self.reference,32767),self.theta_dot)
                u=(sum(k*x for k,x in zip(g,state))+4096)//8192
                if self.mode==3:u+=(self.iq+(1<<(IQ_SH-1)))//(1<<IQ_SH)
            else:
                u=0
            u=int(clip(u,9500))
            uc=int(clip(u+(values[4] if u>0 else -values[5] if u<0 else 0),9500))
            self.duty=int(clip(self.duty+clip(uc-self.duty,3000),9500))
            self.prev_mode=self.mode
        self.integrate(self.torque_for(self.duty,self.mode,self.fault))

    def snapshot(self, revision):
        # 记录格式 v3 的诊断量：模拟器不建模控制器内部，给 0 而不是 None，
        # 保证 SNAPSHOT.pack 的类型与实板一致（None 会让 struct 报错）。
        # v4 追加的 term_k1..k4 / theta_e 同理（控制器内部量，不建模）。
        return dict(tick=self.ticks,revision=revision,mode=self.mode,fault=self.fault,
                    alpha=self.alpha,alpha_dot=self.alpha_dot,theta=self.theta,theta_dot=self.theta_dot,
                    duty=self.duty,reference=self.reference,target=self.target,encoder=self.encoder,
                    adc=self.adc,offset=self.offset,calib_done=self.calib_done,
                    run_en=int(not self.fault and self.mode in (1,2,3)),
                    pos_trim=0,alpha_pre=self.alpha,k3_eff=0,k4_eff=0,
                    duty_pre_dz=self.duty,obs_active=0,alpha_trim=0,
                    term_k1=0,term_k2=0,term_k3=0,term_k4=0,theta_e=0)
