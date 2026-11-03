param(
    [string]$LccRoot = 'C:\Program Files\MATLAB\R2025b\sys\lcc64\lcc64',
    [string]$SdkRoot = 'C:\Users\Sinx\Documents\Codex\hpm_sdk_env_v1.9.0'
)
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$out=Join-Path $root '.cache/pwm_adc_timing'
New-Item -ItemType Directory -Force $out | Out-Null
# 直接测试当前源文件函数；不在测试中另写一份换算实现。
$source=Get-Content -LiteralPath (Join-Path $root 'app/src/pwm.c') -Raw
$driver=Get-Content -LiteralPath (Join-Path $SdkRoot 'hpm_sdk/drivers/inc/hpm_pwmv2_drv.h') -Raw
$timing=[regex]::Match($source,'(?ms)^int pwm_get_adc_trigger_timing\([^\r\n]+\)\r?\n\{.*?^\}')
$getter=[regex]::Match($driver,'(?ms)^static inline uint32_t pwmv2_get_counter_value\([^\r\n]+\)\r?\n\{.*?^\}')
if(-not $timing.Success -or -not $getter.Success){throw 'PWM timing or SDK counter getter not found'}
[IO.File]::WriteAllText((Join-Path $out 'pwm_adc_timing_under_test.inc'),($getter.Value+"`n"+$timing.Value),(New-Object Text.UTF8Encoding($false)))
$flags=@(('-I'+(Join-Path $LccRoot 'include64')),('-I'+$out),('-I'+(Join-Path $root 'motor/inc')),('-I'+(Join-Path $SdkRoot 'hpm_sdk/soc/HPM6E00/ip')))
& (Join-Path $LccRoot 'bin/lcc64.exe') @flags (Join-Path $PSScriptRoot 'test_pwm_adc_timing.c') '-o' (Join-Path $out 'test.obj')
if($LASTEXITCODE -ne 0){throw 'PWM ADC timing host compilation failed'}
& (Join-Path $LccRoot 'bin/lcclnk64.exe') ('-L'+(Join-Path $LccRoot 'lib64')) '-subsystem' console '-o' (Join-Path $out 'test.exe') (Join-Path $out 'test.obj')
if($LASTEXITCODE -ne 0){throw 'PWM ADC timing host linking failed'}
& (Join-Path $out 'test.exe')
if($LASTEXITCODE -ne 0){throw 'PWM ADC timing host test failed'}
