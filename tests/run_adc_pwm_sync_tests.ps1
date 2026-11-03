param([string]$LccRoot = 'C:\Program Files\MATLAB\R2025b\sys\lcc64\lcc64')
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$out=Join-Path $root '.cache/adc_pwm_sync'
New-Item -ItemType Directory -Force $out | Out-Null
$flags=@(('-I'+(Join-Path $LccRoot 'include64')),('-I'+(Join-Path $PSScriptRoot 'adc_host')),('-I'+(Join-Path $root 'app/inc')),('-I'+(Join-Path $root 'app/src')),('-I'+(Join-Path $root 'motor/inc')))
& (Join-Path $LccRoot 'bin/lcc64.exe') @flags (Join-Path $PSScriptRoot 'test_adc_pwm_sync.c') '-o' (Join-Path $out 'test.obj')
if($LASTEXITCODE -ne 0){throw 'ADC PWM sync host compilation failed'}
& (Join-Path $LccRoot 'bin/lcclnk64.exe') ('-L'+(Join-Path $LccRoot 'lib64')) '-subsystem' console '-o' (Join-Path $out 'test.exe') (Join-Path $out 'test.obj')
if($LASTEXITCODE -ne 0){throw 'ADC PWM sync host linking failed'}
& (Join-Path $out 'test.exe')
if($LASTEXITCODE -ne 0){throw 'ADC PWM sync host test failed'}
