param(
    [ValidateSet('syn','full')][string]$Mode='full',
    # 默认输出到仓库同级的 build_output 目录（可传参覆盖）
    [string]$BuildParent=(Join-Path (Split-Path -Parent $PSScriptRoot) 'build_output')
)
# run_t20k.ps1 — Tang Primer 20K 构建入口（结构同 gowin/run_gowin.ps1）
# 每次构建用独立时间戳目录，绝不覆盖历史产物；产物与源码哈希一一对应。
$ErrorActionPreference='Stop'
$source=Split-Path -Parent $PSScriptRoot
if(-not [IO.Path]::IsPathRooted($BuildParent)){throw 'BuildParent must be an absolute path'}
$buildParent=[IO.Path]::GetFullPath($BuildParent)
$buildPrefix=$buildParent.TrimEnd([char[]]'\/')+[IO.Path]::DirectorySeparatorChar
$buildPath=Join-Path $buildParent ('audit_'+(Get-Date -Format 'yyyyMMdd_HHmmss'))
$resolvedBuild=[IO.Path]::GetFullPath($buildPath)
if(-not $resolvedBuild.StartsWith($buildPrefix,[StringComparison]::OrdinalIgnoreCase)){
    throw 'Build path escaped its parent'
}
if(Test-Path -LiteralPath $resolvedBuild){throw 'Build directory already exists'}
New-Item -ItemType Directory -Path $resolvedBuild | Out-Null
Copy-Item -LiteralPath (Join-Path $source 'rtl') -Destination $resolvedBuild -Recurse
Copy-Item -LiteralPath (Join-Path $source 'cons') -Destination $resolvedBuild -Recurse

# 主频：共享 RTL 的 j280_defs.vh 默认 50MHz（DE10）。T20K 板载 27MHz 晶振直驱
# （2026-10-27 降档：rPLL 已移除，时序余量 ±1.5%→约 85%，PnR 不再掷骰）。
# 只改构建快照副本，不动源码树，保证 --check 可复现。
$defsPath=Join-Path $resolvedBuild 'rtl\j280_defs.vh'
$defsText=Get-Content -Raw -LiteralPath $defsPath
$clockDefault='`define J280_CLK_HZ   50_000_000'
if(-not $defsText.Contains($clockDefault)){throw 'Cannot locate shared clock default in j280_defs.vh'}
$defsText=$defsText.Replace($clockDefault,'`define J280_CLK_HZ   27_000_000')
[IO.File]::WriteAllText($defsPath,$defsText,[Text.UTF8Encoding]::new($false))

# 串口波特率 = 源码默认 1M（2026-10-09 定案）：串口已改走外接 CH340（PMOD2
# B12/C12），与 DE10/J280 完全同参。曾在此把快照补丁成 2Mbaud 以适配 BL702
# 桥，现 Type-C 专职 JTAG，补丁撤销——顺带消除该补丁给快照引入的 UTF-8 BOM
# （会噎住 iverilog 仿真）。若将来要回 2M，仅需对快照 j280_defs.vh 重做
# 1_000_000→2_000_000 替换（上位机 devices.py 两个波特率都会自动试）。

# 板标识/构建戳/能力位按本次构建目标重新生成，写在构建目录内（不动源码树）：
# HELLO 靠它告诉上位机"我是哪块板、会什么"。T20K V1 恒为锁步（hil_use=1），
# 上位机据此自动选「锁步HIL」后端，不需要人工记哪块板该用哪个后端。
$python=Join-Path $source '.venv\Scripts\python.exe'
if(-not (Test-Path -LiteralPath $python)){ $python='python' }
& $python (Join-Path $source 'tools\generate_debug_config.py') --board t20k `
    --out (Join-Path $resolvedBuild 'rtl\j280_debug_params.vh')
if($LASTEXITCODE -ne 0){throw 'generate_debug_config.py failed'}

$entry='build.tcl'
$tcl=Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot $entry)
$asciiBuild=$resolvedBuild.Replace('\','/')
$tcl=$tcl.Replace('../rtl/',$asciiBuild+'/rtl/').Replace('../cons/',$asciiBuild+'/cons/')
[IO.File]::WriteAllText((Join-Path $resolvedBuild $entry),$tcl,[Text.UTF8Encoding]::new($false))
Get-ChildItem -LiteralPath (Join-Path $resolvedBuild 'rtl') -File |
    Get-FileHash -Algorithm SHA256 |
    Select-Object @{Name='File';Expression={Split-Path -Leaf $_.Path}},Hash |
    ConvertTo-Json | Set-Content -LiteralPath (Join-Path $resolvedBuild 'source_sha256.json') -Encoding utf8
Write-Output "BUILD_DIRECTORY=$resolvedBuild"
Push-Location -LiteralPath $resolvedBuild
try {
    & 'E:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe' $entry 2>&1 |
        Tee-Object -FilePath (Join-Path $resolvedBuild 'build.log')
    if($LASTEXITCODE -ne 0){throw "Gowin failed: $LASTEXITCODE"}
    if($Mode -eq 'full'){
        # 时序验收：目标频率必须收敛且零违约（报告 HTML 取 Fmax 与违约端点计数）
        # 2026-10-27 主频降档：T20K 现为 27MHz 晶振直驱（原 50.143MHz rPLL）。
        # 门槛随之改为 27MHz——这才是真正需要满足的约束周期；报告里的 Fmax
        # 只是当次布线的能力值（实测仍可达 ~43MHz，即有 ~59% 余量）。
        $timing=Join-Path $resolvedBuild 't20k\impl\pnr\t20k_tr_content.html'
        $html=Get-Content -Raw -LiteralPath $timing
        $text=[regex]::Replace([regex]::Replace($html,'<[^>]+>',' '),'\s+',' ')
        $freq=[regex]::Matches($text,'(\d+\.\d+)\(MHz\)')
        if($freq.Count -lt 2){throw 'Cannot parse timing frequency'}
        $actual=[double]$freq[1].Groups[1].Value
        $setup=[regex]::Match($text,'Numbers of Setup Violated Endpoints (\d+)')
        $hold=[regex]::Match($text,'Numbers of Hold Violated Endpoints (\d+)')
        if(-not $setup.Success -or -not $hold.Success){throw 'Cannot parse timing violations'}
        if($actual -lt 27 -or [int]$setup.Groups[1].Value -ne 0 -or [int]$hold.Groups[1].Value -ne 0){
            throw "Timing failed: Fmax=$actual MHz (target 27 MHz, setup=$($setup.Groups[1].Value), hold=$($hold.Groups[1].Value))"
        }
        Write-Output "TIMING PASS: Fmax=$actual MHz, 0 violations"
        $bin=Join-Path $resolvedBuild 't20k\impl\pnr\t20k.fs'
        if(-not (Test-Path -LiteralPath $bin)){throw 'Bitstream (.fs) missing'}
        Write-Output "BITSTREAM=$bin"
    }
} finally { Pop-Location }
