# build.tcl - Tang Primer 20K (GW2A-LV18PG256C8/I7) full build.
# NOTE: ASCII-only on purpose. gw_sh reads this file as ANSI/GBK; UTF-8 Chinese
# comments can corrupt line structure and swallow the create_project command
# (reproduced 2026-09-21: project had no target device).
# Shared control core (same source as J280/DE10 boards).
create_project -force -name t20k -pn GW2A-LV18PG256C8/I7 -device_version C -dir .
# T10(reset) is SSPI pin; A13/C13(led) are READY/DONE pins - reuse as user IO
# (same practice as Sipeed official LiteX board definition and J280 T20 SW4)
set_option -use_sspi_as_gpio 1
set_option -use_ready_as_gpio 1
set_option -use_done_as_gpio 1
add_file ../rtl/angle_proc.v
add_file ../rtl/arm_vel.v
add_file ../rtl/cnt2deg.v
add_file ../rtl/loop_timer.v
add_file ../rtl/lqr_core.v
add_file ../rtl/mode_fsm.v
add_file ../rtl/motor_pwm.v
add_file ../rtl/out_stage.v
add_file ../rtl/qenc4x.v
add_file ../rtl/rev_guard.v
add_file ../rtl/safety.v
add_file ../rtl/swingup_core.v
add_file ../rtl/traj_gen.v
add_file ../rtl/uart_rx.v
add_file ../rtl/uart_tx.v
add_file ../rtl/ui.v
add_file ../rtl/adc_if.v
add_file ../rtl/ads1220_if.v
# MT6701 SSI alpha sensor (V3 reference; V5 repurposed as motor-side theta sensor)
add_file ../rtl/mt6701_ssi_if.v
# Motor-side MT6701 absolute angle -> arm theta/theta_dot (gear 20.45, V5)
add_file ../rtl/theta_mt_path.v
# MT6835 SPI alpha sensor (V4 active path)
add_file ../rtl/mt6835_spi_if.v
# 4-state steady-state Kalman observer (gains header included, not compiled standalone)
add_file ../rtl/obs_core.v
add_file ../rtl/packet_rx.v
add_file ../rtl/packet_tx.v
add_file ../rtl/debug_link.v
add_file ../rtl/ctrl_core.v
# T20K board shell (rPLL 27->50MHz; rPLL primitive comes from Gowin library)
add_file ../rtl/boards/t20k/t20k_top.v
add_file ../cons/t20k.cst
add_file ../cons/t20k.sdc
set_option -top_module t20k_top
run all
