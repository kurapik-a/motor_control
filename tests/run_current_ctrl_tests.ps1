param(
    [string]$LccRoot = 'C:\Program Files\MATLAB\R2025b\sys\lcc64\lcc64',
    [string]$SdkRoot = 'C:\Users\Sinx\Documents\Codex\hpm_sdk_env_v1.9.0'
)
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$out=Join-Path $root '.cache/current_ctrl'
$vofa=Join-Path $SdkRoot 'hpm_apps/middleware/vofa_debug'
New-Item -ItemType Directory -Force $out | Out-Null
$flags=@(('-I'+(Join-Path $LccRoot 'include64')),('-I'+(Join-Path $PSScriptRoot 'vofa_host')),('-I'+(Join-Path $PSScriptRoot 'host')),('-I'+(Join-Path $root 'app/inc')),('-I'+(Join-Path $root 'motor/inc')),('-I'+(Join-Path $vofa 'inc')),('-I'+(Join-Path $vofa 'src')),('-I'+(Join-Path $vofa 'port/hpm_uart')))
$sources=@('tests/test_current_ctrl.c','tests/vofa_protocol_host.c','motor/src/pmsm_init.c','motor/src/pmsm_current_ctrl.c','motor/src/pmsm_speed_ctrl.c','motor/src/pmsm_foc.c','motor/src/pid.c','motor/src/pmsm_protect.c','app/src/motor_vofa.c')
$objects=@()
foreach($source in $sources) {
    $obj=Join-Path $out (([IO.Path]::GetFileNameWithoutExtension($source))+'.obj')
    & (Join-Path $LccRoot 'bin/lcc64.exe') @flags (Join-Path $root $source) '-o' $obj
    if($LASTEXITCODE -ne 0){throw "Current-loop compilation failed: $source"}
    $objects+=$obj
}
& (Join-Path $LccRoot 'bin/lcclnk64.exe') ('-L'+(Join-Path $LccRoot 'lib64')) '-subsystem' console '-o' (Join-Path $out 'test.exe') @objects
if($LASTEXITCODE -ne 0){throw 'Current-loop linking failed'}
& (Join-Path $out 'test.exe')
if($LASTEXITCODE -ne 0){throw 'Current-loop integration test failed'}
