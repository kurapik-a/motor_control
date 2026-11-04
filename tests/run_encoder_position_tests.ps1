param([string]$LccRoot = 'C:\Program Files\MATLAB\R2025b\sys\lcc64\lcc64')
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$out=Join-Path $root '.cache/encoder_position'
New-Item -ItemType Directory -Force $out | Out-Null
$source=[IO.File]::ReadAllText((Join-Path $root 'app/src/encoder.c'))
$function=[regex]::Match($source,'(?ms)^int encoder_read_position\(int32_t \*position_count\)\r?\n\{.*?^\}')
if(-not $function.Success){throw 'Cannot extract actual encoder_read_position'}
[IO.File]::WriteAllText((Join-Path $out 'encoder_position_under_test.inc'),$function.Value)
$flags=@(('-I'+(Join-Path $LccRoot 'include64')),('-I'+$out))
& (Join-Path $LccRoot 'bin/lcc64.exe') @flags (Join-Path $PSScriptRoot 'test_encoder_position.c') '-o' (Join-Path $out 'test.obj')
if($LASTEXITCODE -ne 0){throw 'Encoder position compilation failed'}
& (Join-Path $LccRoot 'bin/lcclnk64.exe') ('-L'+(Join-Path $LccRoot 'lib64')) '-subsystem' console '-o' (Join-Path $out 'test.exe') (Join-Path $out 'test.obj')
if($LASTEXITCODE -ne 0){throw 'Encoder position linking failed'}
& (Join-Path $out 'test.exe')
if($LASTEXITCODE -ne 0){throw 'Encoder position tests failed'}
