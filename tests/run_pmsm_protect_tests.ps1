param([string]$LccRoot = 'C:\Program Files\MATLAB\R2025b\sys\lcc64\lcc64')
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$out=Join-Path $root '.cache/pmsm_protect'
New-Item -ItemType Directory -Force $out | Out-Null
$baseFlags=@(('-I'+(Join-Path $LccRoot 'include64')),('-I'+(Join-Path $root 'motor/inc')))
foreach($mode in @('default','dq_speed','lower_limits','dq_off')) {
    $flags=@($baseFlags)
    if($mode -eq 'dq_speed') { $flags+=@('-DMOTOR_PROTECT_DQ_ENABLE=1','-DMOTOR_PROTECT_SPEED_ERROR_ENABLE=1') }
    if($mode -eq 'lower_limits') { $flags+=@('-DMOTOR_BUS_UNDERVOLTAGE_ADC=100u','-DMOTOR_TEMP_MIN_ADC=200u') }
    if($mode -eq 'dq_off') { $flags+='-DMOTOR_PROTECT_DQ_ENABLE=0' }
    & (Join-Path $LccRoot 'bin/lcc64.exe') @flags (Join-Path $root 'motor/src/pmsm_protect.c') '-o' (Join-Path $out "$mode-module.obj")
    if($LASTEXITCODE -ne 0){throw "Protection module compilation failed: $mode"}
    & (Join-Path $LccRoot 'bin/lcc64.exe') @flags (Join-Path $PSScriptRoot 'test_pmsm_protect.c') '-o' (Join-Path $out "$mode-test.obj")
    if($LASTEXITCODE -ne 0){throw "Protection test compilation failed: $mode"}
    & (Join-Path $LccRoot 'bin/lcclnk64.exe') ('-L'+(Join-Path $LccRoot 'lib64')) '-subsystem' console '-o' (Join-Path $out "$mode.exe") (Join-Path $out "$mode-test.obj") (Join-Path $out "$mode-module.obj")
    if($LASTEXITCODE -ne 0){throw "Protection linking failed: $mode"}
    & (Join-Path $out "$mode.exe")
    if($LASTEXITCODE -ne 0){throw "Protection test failed: $mode"}
}
