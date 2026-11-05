param([string]$LccRoot = 'C:\Program Files\MATLAB\R2025b\sys\lcc64\lcc64')
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$out=Join-Path $root '.cache/current_ctrl/trajectory'
New-Item -ItemType Directory -Force $out | Out-Null
$flags=@(('-I'+(Join-Path $LccRoot 'include64')),('-I'+(Join-Path $root 'motor/inc')),('-I'+(Join-Path $root 'app/inc')),('-I'+(Join-Path $PSScriptRoot 'host')))
foreach($variant in @('default','jerk_limited_peak_speed')) {
    $variantFlags=$flags
    if($variant -eq 'jerk_limited_peak_speed') { $variantFlags+=('-DMOTOR_TRAJECTORY_JERK_COUNTS_S3=1000.0f') }
    $objects=@()
    foreach($source in @('tests/test_pmsm_trajectory.c','motor/src/pmsm_trajectory.c')) {
        $obj=Join-Path $out (([IO.Path]::GetFileNameWithoutExtension($source))+'.obj')
        & (Join-Path $LccRoot 'bin/lcc64.exe') @variantFlags (Join-Path $root $source) '-o' $obj
        if($LASTEXITCODE -ne 0){throw "Trajectory compilation failed: $source"}
        $objects+=$obj
    }
    & (Join-Path $LccRoot 'bin/lcclnk64.exe') ('-L'+(Join-Path $LccRoot 'lib64')) '-subsystem' console '-o' (Join-Path $out 'trajectory_test.exe') @objects
    if($LASTEXITCODE -ne 0){throw 'Trajectory linking failed'}
    Write-Output "Trajectory variant: $variant"
    & (Join-Path $out 'trajectory_test.exe')
    if($LASTEXITCODE -ne 0){throw 'Trajectory test failed'}
}
