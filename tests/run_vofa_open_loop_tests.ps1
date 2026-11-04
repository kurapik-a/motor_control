param(
    [string]$LccRoot = 'C:\Program Files\MATLAB\R2025b\sys\lcc64\lcc64',
    [string]$SdkRoot = 'C:\Users\Sinx\Documents\Codex\hpm_sdk_env_v1.9.0'
)
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$out=Join-Path $root '.cache/vofa_open_loop'
$vofa=Join-Path $SdkRoot 'hpm_apps/middleware/vofa_debug'
New-Item -ItemType Directory -Force $out | Out-Null
$flags=@(('-I'+(Join-Path $LccRoot 'include64')),('-I'+(Join-Path $PSScriptRoot 'vofa_host')),('-I'+(Join-Path $PSScriptRoot 'host')),('-I'+(Join-Path $root 'app/inc')),('-I'+(Join-Path $root 'app/src')),('-I'+(Join-Path $root 'motor/inc')),('-I'+(Join-Path $vofa 'inc')),('-I'+(Join-Path $vofa 'src')),('-I'+(Join-Path $vofa 'port/hpm_uart')))
$flags+='-DMOTOR_CONTROL_USE_CURRENT_LOOP=0'
& (Join-Path $LccRoot 'bin/lcc64.exe') @flags (Join-Path $PSScriptRoot 'test_vofa_open_loop.c') '-o' (Join-Path $out 'test.obj')
if($LASTEXITCODE -ne 0){throw 'VOFA open-loop compilation failed'}
& (Join-Path $LccRoot 'bin/lcc64.exe') @flags (Join-Path $PSScriptRoot 'vofa_protocol_host.c') '-o' (Join-Path $out 'vofa.obj')
if($LASTEXITCODE -ne 0){throw 'VOFA protocol compilation failed'}
& (Join-Path $LccRoot 'bin/lcclnk64.exe') ('-L'+(Join-Path $LccRoot 'lib64')) '-subsystem' console '-o' (Join-Path $out 'test.exe') (Join-Path $out 'test.obj') (Join-Path $out 'vofa.obj')
if($LASTEXITCODE -ne 0){throw 'VOFA open-loop linking failed'}
& (Join-Path $out 'test.exe')
if($LASTEXITCODE -ne 0){throw 'VOFA open-loop test failed'}
