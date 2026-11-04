#!/bin/bash
# ============================================================================
# run_sim.sh — iverilog 仿真跑批（Git Bash / Linux）
# 用法: ./run_sim.sh [testname]     不带参数 = 全部
# debug_uart / uart_mission 由 host 包驱动：Python 生成字节级激励、编译、
# 经真实 UART 引脚运行并对每条确认与遥测帧解码判据。uart_mission 为按需
# 扩展项（真实闭环任务，iverilog 需数分钟）：./run_sim.sh uart_mission。
# ============================================================================
set -u
cd "$(dirname "$0")/.." || exit 1
RTL=rtl
mkdir -p sim/out

python_bin() {
  if [ -x ".venv/Scripts/python.exe" ]; then echo ".venv/Scripts/python.exe"
  elif [ -x ".venv/bin/python" ]; then echo ".venv/bin/python"
  else echo "python"; fi
}

driver_for() {
  case "$1" in
    debug_uart) echo host.test_rtl_uart ;;
    uart_mission) echo host.test_rtl_mission ;;
    *) echo "" ;;
  esac
}

run_one() {
    local name="$1"
    echo "=== $name ==="
    local driver
    driver=$(driver_for "$name")
    if [ -n "$driver" ]; then
        "$(python_bin)" -m "$driver"
        return $?
    fi
    iverilog -g2012 -I "$RTL" -s "tb_${name}" -o "sim/out/${name}.vvp" \
        "tb/tb_${name}.v" "$RTL"/*.v "$RTL"/boards/*/*.v 2>&1
    if [ $? -ne 0 ]; then echo ">>> COMPILE FAIL: $name"; return 1; fi
    local result dip
    if [ "$name" = "closedloop" ]; then
        for dip in 0 1 2 3; do
            result=$(vvp "sim/out/${name}.vvp" "+DIP=$dip" "+TRACE=sim/out/${name}_dip${dip}.csv") ||
                { echo "$result"; return 1; }
            echo "$result" | tee "sim/out/${name}_dip${dip}.log"
            [[ "$result" == *"tb_${name} PASS"* && "$result" != *"FAIL"* && "$result" != *"TIMEOUT"* ]] || return 1
        done
    else
        result=$(vvp "sim/out/${name}.vvp") || { echo "$result"; return 1; }
        echo "$result"
        [[ "$result" == *"tb_${name} PASS"* && "$result" != *"FAIL"* && "$result" != *"TIMEOUT"* ]]
    fi
}

# 全量编译整个 rtl 目录（模块互不冲突），并行跑独立进程
if [ $# -ge 1 ]; then
    run_one "$1"
else
    fail=0
    pids=""
    tests="top_smoke closedloop debug_uart control_path arm_vel units adc_if lqr_core qenc4x"
    for t in $tests; do
        run_one "$t" > "sim/out/${t}.log" 2>&1 &
        pids="$pids $!"
    done
    rc=0
    for p in $pids; do wait "$p" || rc=1; done
    for t in $tests; do
        cat "sim/out/${t}.log"
    done
    echo "================================"
    [ $rc -eq 0 ] && echo "ALL SIM PASS" || echo "SIM FAILURES PRESENT"
    exit "$rc"
fi
